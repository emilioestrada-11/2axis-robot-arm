SCARA Robot Arm — Desktop Controller & Real-Time Visualizer
A Python desktop application for controlling a 2-DOF SCARA robot arm via ESP32. The GUI provides real-time inverse kinematics visualization, G-code execution, and live serial communication — so you can see exactly where the arm is at all times.

Features:
Real-time 2D visualization — arm links, joints, and end-effector update live as the robot moves, driven by forward and inverse kinematics
G-code executor — supports G0/G1 motion, G90/G91 (absolute/relative), G20/G21 (inch/mm), and M2 program end
Shape commands — send pre-built Square, Circle, and Triangle paths with one click
Live joint readout — θ1 and θ2 displayed in degrees alongside the end-effector (X, Y) position
End-effector trail — fading path history shows where the arm has been
Line-by-line serial sync — each G-code line waits for ESP32 acknowledgment before sending the next, preventing command overflow


Hardware Requirements:
ComponentDetailsMicrocontrollerESP32 (MicroPython firmware)
Link lengthsL1 = 140 mm, L2 = 140 mm
Base offsetX = −100 mm, Y = 0 mm
MotorsServo or stepper (2 joints)
ConnectionUSB serial, 115200 baud

Software Requirements:
bashPython 3.8+
pip install pyserial
MicroPython must be flashed onto the ESP32. The firmware file is located at micropython/main.py.

How to Run:
Flash micropython/main.py to the ESP32
Connect the ESP32 via USB and note the COM port (e.g. COM3)
Run GUI
