# Autonomous RC Car with Dual OpenMV Cameras + Custom PCB

This project turns a standard RC car into an autonomous vehicle using two OpenMV Cam H7 microcontrollers, a custom motor control PCB, and onboard computer vision and control algorithms. The car can detect lane markings, adjust steering with PID control, and respond to stop signs or traffic lights autonomously.

---

## Features

- **Dual-Camera System**:
  - **Primary OpenMV** handles real-time lane detection and PID steering control.
  - **Secondary OpenMV** identifies traffic signs and colored blobs (stop signs, blue lights) and sends commands via UART.

- **PID-Based Steering**:
  - Calculates deflection angle using lane boundaries.
  - Dynamically adjusts servo angle to stay centered.
  - Full support for proportional, derivative, and (optional) integral control.

- **Custom Motor Control PCB**:
  - Designed in Altium.
  - Drives motor via H-bridge with PWM + GPIO.
  - Distributes 5V to OpenMV boards and sensors from a main battery input.

- **Real-World Design Considerations**:
  - Custom 3D-printed mounts for optimal camera angles.
  - Wide-angle lens added to improve field of view.
  - Region-of-interest optimizations for visual processing.

---

## Software Overview

### `main_camera/main.py`
- Detects left and right lane boundaries using `find_lines()`.
- Computes lane center and deflection angle.
- Applies PID corrections to steer the servo motor.
- Receives UART commands (`'R'`, `'B'`) from the secondary camera to control movement logic.

### `secondary_camera/main.py`
- Detects stop signs (red) and traffic lights (blue) using color blob filtering.
- Sends corresponding control commands to the main camera over UART.

---

## Hardware Overview

### Components:
- 2x OpenMV Cam H7 R1
- Custom Motor Control PCB
  - VNH5019A-E (Motor Driver H-Bridge)
  - AMS1117-5.0 (LDO Voltage Regulator)
- Standard RC car chassis
- Servo motor (steering)
- DC motor (driving)
- Ultrasonic sensor (optional)

### PCB:
- Located in `pcb_design/`:
  - `Motor_Control_PCB_Schematic.SchDoc`
  - `Motor_Control_PCB.PcbDoc`

- Supplies regulated 5V to OpenMV boards and sensors.
- Implements motor H-bridge control using PWM and direction pins.

---

## Project Structure

```bash
autonomous-rc-car-openmv/
├── main_camera/
│   └── main.py                # Lane detection + PID control
├── secondary_camera/
│   └── main.py                # Stop sign / traffic light detection
├── pcb_design/
│   ├── Motor_Control_PCB_Schematic.SchDoc
│   └── Motor_Control_PCB.PcbDoc
├── media/
│   └──
├── README.md
```

---

## Getting Started

### Flashing Code
1. Connect each OpenMV Cam H7 to your computer.
2. Use the [OpenMV IDE](https://openmv.io/pages/download) to flash:
   - `main_camera/main.py` to the lane-following camera.
   - `secondary_camera/main.py` to the stop sign/traffic camera.

### Hardware Hookups
- Follow the circuit diagram and use the PCB to route motor power, servo PWM, and UART between cameras.

---

## Contributors

Team 11 – UC Davis EEC195B (Spring 2023)  
- Allen Benjamin  
- Edwin Munguia  
- Hugo Wong  
- Rene Lim

---

## License

MIT License – Feel free to use and build on this work.
---

## What I Learned & Challenges Faced

This project, part of my UC Davis senior design course, helped me understand the realities of building a hardware-software integrated autonomous system from the ground up. Some major lessons and hurdles that were encountered include:

- **Choosing the Right Platform**: Initially, we experimented with a Jetson Nano for image processing, but due to severe frame delays, we pivoted to a dual OpenMV setup that would expand the pipeline resulting in faster, low-latency computer vision.

- **Lane Detection Complexity**: Implementing reliable lane detection required extensive experimentation with regions of interest, binary thresholds, and camera angles. The PID constants (Kp, Kd, Ki) had to be fine-tuned repeatedly to handle real-world inconsistencies like lighting and curvature.

- **Field of View Limitations**: The default OpenMV lens offered a limited field of view. Accuracy was improved by adding a wide-angle lens and designing custom 3D mounts to position the camera above the track.

- **Motor Control & Electrical Design**: Building a custom PCB to drive the motor required understanding H-bridge logic, power regulation, and noise considerations. Debugging voltage drops and ensuring consistent servo performance was a hands-on challenge using tools such as multimeters and oscilloscopes.

- **Object Detection Trade-offs**: Differentiating traffic signs by shape and color proved harder than expected due to lighting variability. Tuned color thresholds and used blob filtering but was unable to fully implement multiple shape classification due to time constraints; the system was only able to recognize a stop sign.

Through these challenges, I gained a strong appreciation for embedded systems integration, automation, feedback control systems, hardware debugging, and real-time computer vision. This project deepened my understanding of what it takes to build intelligent robotics systems under real-world constraints.
