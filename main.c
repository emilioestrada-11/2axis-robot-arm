import machine
import time
import math
import sys
import select

print("SCARA ROBOT CODE RUNNING")
print("READY")

# -----------------------------
# Robot arm geometry
# -----------------------------
L1 = 140
L2 = 140
BASE_X = -100
BASE_Y = 0

STEPS_PER_RAD = 650

# Speed configuration
TRAVEL_SPEED_MM_PER_S = 101.6  # fast for repositioning

# Per-shape speeds — adjust these to tune each shape independently
LINE_SPEED     = 75.0   # mm/s
SQUARE_SPEED   = 25.0   # mm/s - slow for accuracy
TRIANGLE_SPEED = 40.0   # mm/s
CIRCLE_SPEED   = 30.0   # mm/s - slow for smooth curves

DRAW_SPEED_MM_PER_S = 50.0  # fallback default

# -----------------------------
# Stepper motor pins
# -----------------------------
m1_pins = [
    machine.Pin(26, machine.Pin.OUT),
    machine.Pin(27, machine.Pin.OUT),
    machine.Pin(14, machine.Pin.OUT),
    machine.Pin(12, machine.Pin.OUT)
]

m2_pins = [
    machine.Pin(32, machine.Pin.OUT),
    machine.Pin(33, machine.Pin.OUT),
    machine.Pin(25, machine.Pin.OUT),
    machine.Pin(13, machine.Pin.OUT)
]

sequence = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]
]

# -----------------------------
# LEDs
# -----------------------------
led_idle  = machine.Pin(17, machine.Pin.OUT)
led_draw  = machine.Pin(4,  machine.Pin.OUT)
led_estop = machine.Pin(15, machine.Pin.OUT)

# -----------------------------
# Buttons
# -----------------------------
btn_line     = machine.Pin(23, machine.Pin.IN, machine.Pin.PULL_UP)
btn_square   = machine.Pin(22, machine.Pin.IN, machine.Pin.PULL_UP)
btn_triangle = machine.Pin(21, machine.Pin.IN, machine.Pin.PULL_UP)
btn_circle   = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
btn_estop    = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)

# -----------------------------
# Global state
# -----------------------------
emergency_stop_active = False
drawing_in_progress   = False
current_shape         = None

# Joint angle tracking (radians)
current_theta1 = 0.0
current_theta2 = 0.0

# XY position tracking
current_x = 0.0
current_y = 0.0

# G-code modal state (updated by G20/G21/G90/G91 received from GUI)
gcode_absolute = True   # G90 = True, G91 = False
gcode_units_mm = True   # G21 = True, G20 = False

# -----------------------------
# Stepper control
# -----------------------------
step_index1 = 0
step_index2 = 0

def step_motor(pins, step_index):
    pattern = sequence[step_index % 8]
    for i in range(4):
        pins[i].value(pattern[i])

def step1(direction):
    global step_index1
    if not emergency_stop_active:
        step_index1 = (step_index1 + direction) % 8
        step_motor(m1_pins, step_index1)

def step2(direction):
    global step_index2
    if not emergency_stop_active:
        step_index2 = (step_index2 + direction) % 8
        step_motor(m2_pins, step_index2)

def disable_motors():
    for pin in m1_pins:
        pin.value(0)
    for pin in m2_pins:
        pin.value(0)

# -----------------------------
# Inverse kinematics
# -----------------------------
def inverse_kinematics(x, y):
    rel_x = x - BASE_X
    rel_y = y - BASE_Y

    d = (rel_x*rel_x + rel_y*rel_y - L1*L1 - L2*L2) / (2*L1*L2)

    if abs(d) > 1:
        return None

    theta2 = math.acos(d)

    if abs(math.sin(theta2)) < 1e-6:
        return None

    theta1 = math.atan2(rel_y, rel_x) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2)
    )

    return theta1, theta2

# -----------------------------
# Step delay calculation
# -----------------------------
def calculate_step_delay(distance_mm, num_steps, speed):
    if num_steps == 0:
        return 2
    time_needed_ms = (distance_mm / speed) * 1000
    delay_ms = time_needed_ms / num_steps
    return max(1, min(delay_ms, 10))

# -----------------------------
# Move robot to XY with speed control - FIXED for simultaneous movement
# -----------------------------
def move_to(x, y, speed=None):
    global emergency_stop_active, current_theta1, current_theta2, current_x, current_y

    if speed is None:
        speed = TRAVEL_SPEED_MM_PER_S

    if emergency_stop_active:
        return

    angles = inverse_kinematics(x, y)

    if not angles:
        print("UNREACHABLE: ({}, {})".format(x, y))
        return

    a1, a2 = angles

    delta1 = a1 - current_theta1
    delta2 = a2 - current_theta2

    steps1 = int(delta1 * STEPS_PER_RAD)
    steps2 = int(delta2 * STEPS_PER_RAD)

    dir1 = 1 if steps1 >= 0 else -1
    dir2 = 1 if steps2 >= 0 else -1

    steps1 = abs(steps1)
    steps2 = abs(steps2)

    max_steps = max(steps1, steps2)
    
    # Calculate step ratios for interpolation
    if max_steps > 0:
        ratio1 = steps1 / max_steps
        ratio2 = steps2 / max_steps
    else:
        ratio1 = ratio2 = 0

    dx = x - current_x
    dy = y - current_y
    distance = math.sqrt(dx*dx + dy*dy)
    step_delay = calculate_step_delay(distance, max_steps, speed)
    
    # Track how many steps each motor has taken
    steps_taken1 = 0
    steps_taken2 = 0
    
    # Variables for Bresenham-style interpolation
    error1 = 0
    error2 = 0

    for i in range(max_steps):
        if emergency_stop_active:
            print("MOVEMENT STOPPED - Emergency stop activated")
            return

        if btn_estop.value() == 0 and not emergency_stop_active:
            emergency_stop()
            return

        # Determine if motor 1 should step this iteration
        should_step1 = False
        if steps_taken1 < steps1:
            # Using Bresenham's line algorithm for smooth interpolation
            if steps1 > 0:
                error1 += steps1
                if error1 >= max_steps:
                    error1 -= max_steps
                    should_step1 = True
        
        # Determine if motor 2 should step this iteration
        should_step2 = False
        if steps_taken2 < steps2:
            if steps2 > 0:
                error2 += steps2
                if error2 >= max_steps:
                    error2 -= max_steps
                    should_step2 = True
        
        # Step both motors simultaneously if needed
        if should_step1:
            step1(dir1)
            steps_taken1 += 1
        if should_step2:
            step2(dir2)
            steps_taken2 += 1

        time.sleep_ms(int(step_delay))

    current_theta1 = a1
    current_theta2 = a2
    current_x = x
    current_y = y

    print("POS:{},{}".format(x, y))

# -----------------------------
# Interpolated move helper
# -----------------------------
def interp_move(x0, y0, x1, y1, steps, speed):
    for j in range(1, steps + 1):
        if emergency_stop_active:
            return
        xi = x0 + (x1 - x0) * j / steps
        yi = y0 + (y1 - y0) * j / steps
        move_to(xi, yi, speed=speed)

# -----------------------------
# Home function
# -----------------------------
def home_robot():
    global emergency_stop_active

    if emergency_stop_active:
        print("Cannot home - emergency stop active")
        return

    print("HOMING: Moving to (180,0)")
    led_draw.on()
    led_idle.off()

    move_to(180, 0, speed=TRAVEL_SPEED_MM_PER_S)

    led_draw.off()
    led_idle.on()

    print("POS:180,0")
    print("STATE:IDLE")
    print("HOMING COMPLETE")

# -----------------------------
# Emergency stop
# -----------------------------
def emergency_stop():
    global emergency_stop_active, drawing_in_progress

    print("EMERGENCY STOP ACTIVATED")
    emergency_stop_active = True
    drawing_in_progress   = False

    led_draw.off()
    led_idle.off()
    led_estop.on()

    disable_motors()

    while True:
        while btn_estop.value() == 0:
            time.sleep_ms(50)

        time.sleep_ms(100)

        print("Press E-STOP again to reset")
        while btn_estop.value() == 1:
            time.sleep_ms(50)
            check_serial_for_reset()

        time.sleep_ms(100)

        if btn_estop.value() == 0:
            reset_after_emergency()
            break

def reset_after_emergency():
    global emergency_stop_active, drawing_in_progress, current_shape

    print("RESETTING AFTER EMERGENCY STOP")
    emergency_stop_active = False

    led_estop.off()
    led_idle.on()

    time.sleep_ms(500)

    print("Emergency stop reset.")
    print("  - Press shape button to restart")
    print("  - Send HOME to return to (0,0)")

def check_serial_for_reset():
    if select.select([sys.stdin], [], [], 0)[0]:
        cmd = sys.stdin.readline().strip().upper()
        if cmd in ("RESET", "CONTINUE"):
            reset_after_emergency()
            return True
        elif cmd == "HOME":
            reset_after_emergency()
            home_robot()
            return True
    return False

# -----------------------------
# Shape drawing wrapper
# -----------------------------
def draw_shape(shape_func, shape_name, speed=DRAW_SPEED_MM_PER_S):
    global drawing_in_progress, current_shape, emergency_stop_active

    if emergency_stop_active:
        print("Cannot draw {} - emergency stop active".format(shape_name))
        return

    drawing_in_progress = True
    current_shape       = shape_name

    try:
        shape_func(speed=speed)
    except Exception as e:
        print("Error drawing {}: {}".format(shape_name, e))
    finally:
        if not emergency_stop_active:
            drawing_in_progress = False
            current_shape       = None
            led_draw.off()
            led_idle.on()

# -----------------------------
# Shape definitions
# -----------------------------
def draw_line(speed=LINE_SPEED):
    led_idle.off()
    led_draw.on()
    print("Drawing line at {}mm/s".format(speed))

    total_distance = 254
    num_segments   = 50
    step_size      = total_distance / num_segments

    for i in range(num_segments + 1):
        if emergency_stop_active:
            print("Line interrupted by emergency stop")
            return
        move_to(0, -i * step_size, speed=speed)

def draw_square(speed=SQUARE_SPEED):
    led_idle.off()
    led_draw.on()
    print("Drawing square at {}mm/s".format(speed))

    move_to(0, 0, speed=TRAVEL_SPEED_MM_PER_S)

    pts = [(0,0), (75,0), (75,-75), (0,-75), (0,0)]

    for i in range(len(pts) - 1):
        if emergency_stop_active:
            print("Square interrupted by emergency stop")
            return
        interp_move(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1], 30, speed)

def draw_triangle(speed=TRIANGLE_SPEED):
    led_idle.off()
    led_draw.on()
    print("Drawing triangle at {}mm/s".format(speed))

    pts = [(0,0), (75,-37.5), (0,-75), (0,0)]

    for i in range(len(pts) - 1):
        if emergency_stop_active:
            print("Triangle interrupted by emergency stop")
            return
        interp_move(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1], 30, speed)

def generate_circle_points(radius, center_x, center_y, num_points=36):
    points = []
    for i in range(num_points + 1):
        angle = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        points.append((x, y))
    return points

def draw_circle(speed=CIRCLE_SPEED):
    led_idle.off()
    led_draw.on()
    print("Drawing circle at {}mm/s".format(speed))

    pts = generate_circle_points(37.5, 37.5, 0, 36)

    for p in pts:
        if emergency_stop_active:
            print("Circle interrupted by emergency stop")
            return
        move_to(p[0], p[1], speed=speed)

# -----------------------------
# G-code parser helper
# -----------------------------
def parse_gcode_value(cmd, letter):
    """Extract a float value for a given letter from a G-code string.
    Returns None if the letter is not found."""
    idx = cmd.find(letter)
    if idx == -1:
        return None
    # Collect characters after the letter until a space or end of string
    start = idx + 1
    end = start
    while end < len(cmd) and (cmd[end].isdigit() or cmd[end] == '.' or cmd[end] == '-'):
        end += 1
    try:
        return float(cmd[start:end])
    except ValueError:
        return None

# -----------------------------
# Serial command handler
# -----------------------------
def check_serial():
    global current_shape, gcode_absolute, gcode_units_mm

    if select.select([sys.stdin], [], [], 0)[0]:
        raw = sys.stdin.readline().strip()
        cmd = raw.upper()

        # ----------------------------------------
        # Shape commands
        # ----------------------------------------
        if cmd == "LINE":
            draw_shape(draw_line, "LINE", speed=LINE_SPEED)

        elif cmd == "SQUARE":
            draw_shape(draw_square, "SQUARE", speed=SQUARE_SPEED)

        elif cmd == "TRIANGLE":
            draw_shape(draw_triangle, "TRIANGLE", speed=TRIANGLE_SPEED)

        elif cmd == "CIRCLE":
            draw_shape(draw_circle, "CIRCLE", speed=CIRCLE_SPEED)

        # Speed override e.g. "SQUARE:50"
        elif cmd.startswith("LINE:"):
            draw_shape(draw_line, "LINE", speed=float(cmd.split(":")[1]))
        elif cmd.startswith("SQUARE:"):
            draw_shape(draw_square, "SQUARE", speed=float(cmd.split(":")[1]))
        elif cmd.startswith("TRIANGLE:"):
            draw_shape(draw_triangle, "TRIANGLE", speed=float(cmd.split(":")[1]))
        elif cmd.startswith("CIRCLE:"):
            draw_shape(draw_circle, "CIRCLE", speed=float(cmd.split(":")[1]))

        # ----------------------------------------
        # G-code modal state updates
        # ----------------------------------------
        elif cmd == "G90":
            gcode_absolute = True
            print("STATE:IDLE")

        elif cmd == "G91":
            gcode_absolute = False
            print("STATE:IDLE")

        elif cmd == "G20":
            gcode_units_mm = False
            print("STATE:IDLE")

        elif cmd == "G21":
            gcode_units_mm = True
            print("STATE:IDLE")

        # ----------------------------------------
        # G0 / G1 motion commands
        # ----------------------------------------
        elif cmd.startswith("G0") or cmd.startswith("G1"):
            try:
                x_val = current_x
                y_val = current_y
                if "X" in cmd:
                    xi = cmd.index("X")
                    x_val = float(cmd[xi+1:].split()[0])
                if "Y" in cmd:
                    yi = cmd.index("Y")
                    y_val = float(cmd[yi+1:].split()[0])

                led_draw.on()
                led_idle.off()
                move_to(x_val, y_val, speed=TRAVEL_SPEED_MM_PER_S)
                led_draw.off()
                led_idle.on()
                print("STATE:IDLE")

            except Exception as e:
                print("G-code parse error: {}".format(e))
                print("STATE:IDLE")

        # ----------------------------------------
        # Utility commands
        # ----------------------------------------
        elif cmd == "HOME" and not emergency_stop_active:
            home_robot()

        elif cmd == "ESTOP":
            emergency_stop()

        elif cmd == "DISABLE":
            disable_motors()
            print("Motors disabled")
            print("STATE:IDLE")

        elif cmd == "CONTINUE" and not emergency_stop_active and current_shape:
            print("Continuing {}".format(current_shape))
            shape_map = {
                "LINE":     (draw_line,     LINE_SPEED),
                "SQUARE":   (draw_square,   SQUARE_SPEED),
                "TRIANGLE": (draw_triangle, TRIANGLE_SPEED),
                "CIRCLE":   (draw_circle,   CIRCLE_SPEED),
            }
            if current_shape in shape_map:
                func, spd = shape_map[current_shape]
                draw_shape(func, current_shape, speed=spd)

        elif cmd == "STATUS":
            if emergency_stop_active:
                print("STATE:ESTOP")
            elif drawing_in_progress:
                print("STATE:DRAWING")
            else:
                print("STATE:IDLE")
            print("POS:{},{}".format(current_x, current_y))

        elif cmd == "WHERE":
            print("POS:{},{}".format(current_x, current_y))

        elif cmd in ("M2", "M30"):
            # End of program — nothing to do on the robot side
            print("STATE:IDLE")

        else:
            if cmd:
                print("UNKNOWN CMD: {}".format(cmd))

# -----------------------------
# Startup
# -----------------------------
led_idle.on()
print("SCARA Robot Ready")
print("Arm lengths: L1={}mm, L2={}mm  Base: ({}, {})".format(L1, L2, BASE_X, BASE_Y))
print("Speeds - Line:{}  Square:{}  Triangle:{}  Circle:{} mm/s".format(
    LINE_SPEED, SQUARE_SPEED, TRIANGLE_SPEED, CIRCLE_SPEED))
print("Commands: LINE, SQUARE, TRIANGLE, CIRCLE, HOME, ESTOP, DISABLE, STATUS, WHERE")
print("Speed override syntax: SQUARE:50  (draws square at 50mm/s)")
print("G-code: G0/G1 X<n> Y<n> [F<n>]  — F is in mm/min")

start_angles = inverse_kinematics(180, 0)
if start_angles:
    current_theta1, current_theta2 = start_angles
    current_x = 180.0
    current_y = 0.0
    print("Initial angles set for position (180, 0)")
else:
    print("WARNING: Could not set initial angles for (180,0)")

print("\n--- HOMING ROBOT TO (0,0) ---")
home_robot()
print("--- ROBOT READY ---\n")

# -----------------------------
# Main loop
# -----------------------------
while True:
    check_serial()

    if btn_estop.value() == 0 and not emergency_stop_active:
        emergency_stop()

    if not emergency_stop_active:
        if btn_line.value() == 0:
            print("BUTTON LINE PRESSED")
            draw_shape(draw_line, "LINE", speed=LINE_SPEED)
            while btn_line.value() == 0:
                time.sleep_ms(100)

        if btn_square.value() == 0:
            print("BUTTON SQUARE PRESSED")
            draw_shape(draw_square, "SQUARE", speed=SQUARE_SPEED)
            while btn_square.value() == 0:
                time.sleep_ms(100)

        if btn_triangle.value() == 0:
            print("BUTTON TRIANGLE PRESSED")
            draw_shape(draw_triangle, "TRIANGLE", speed=TRIANGLE_SPEED)
            while btn_triangle.value() == 0:
                time.sleep_ms(100)

        if btn_circle.value() == 0:
            print("BUTTON CIRCLE PRESSED")
            draw_shape(draw_circle, "CIRCLE", speed=CIRCLE_SPEED)
            while btn_circle.value() == 0:
                time.sleep_ms(100)

    time.sleep_ms(50)