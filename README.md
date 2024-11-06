# Manipulator-Robot

This project was developed as part of the Mechatronics System Lab for the WS 2023-24 semester. The goal was to program a robot manipulator using MATLAB to perform automated pick, place, and homing operations between stations A, B, and C.

## Objectives

The primary objective was to control a LEGO MINDSTORMS EV3-based robotic arm remotely in order to:

- Pick and place a ball between predefined stations.
- Implement homing functionality to reset the robot's arm to its default position.
- Utilize inverse kinematics and PID control for accurate motion.

## System Configuration

- **Robot Arm**: Consists of 4 links with lengths of 50mm, 98mm, 185mm, and 110mm.
- **Motors**: Three rotary actuators (Motor A, B, C) control the arm, base, and gripper.
- **Sensors**: Two touch sensors and three encoders provide feedback for motor control and motion limits.
- **Control**: The system is linked to MATLAB, where motor and sensor variables are defined for controlling movement.

## Key Features

- **Kinematics**: Calculated using inverse kinematics equations to ensure precise positioning.
- **PID Control**: Implemented PI (Proportional-Integral) control for base rotation, arm movement, and ball handling. Tuned values for Kp and Ki ensure smooth and stable operation.
- **Task Sequence**: The robot executes pick and place operations, picking the ball from one station and placing it at another in a predefined sequence.
- **Homing Function**: A homing sequence brings the robot arm back to its initial position after each task.

## Project Workflow

1. **Initial Setup**: The robot starts at a random position, then moves to station A using the homing function.
2. **Task Execution**: The robot picks and places the ball between stations A, B, and C based on forward kinematics and sensor readings.
3. **Height Measurement**: Each pick-and-place operation involves height measurement to ensure the ball is lifted and placed at the correct height.
4. **Homing and Reset**: After task completion, the robot returns to its starting position using the homing function.

## References

- Working video: https://drive.google.com/file/d/16Guq8lO1fHBcwLgfZrxdOx4QjjDwkUYl/view?usp=sharing

