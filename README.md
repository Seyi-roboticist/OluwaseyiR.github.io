# OluwaseyiR.github.io 

# UR5 Robot "Place-and-Draw" Task Project

## Introduction

This project focuses on precisely controlling the UR5 robot arm to perform a "place-and-draw" task. Our team has enabled the UR5 to identify start and target locations and automate the task using various control schemes. We have implemented and demonstrated three distinct control trajectories for the UR5 in both simulated and real-world settings, namely:

1. **Inverse Kinematics (IK)**
2. **Resolved-Rate Control Using Differential Kinematics (RR)**
3. **Jacobian Transpose**

## Technologies Used

- **Robot Operating System (ROS):** For managing communication between software components.
- **MATLAB:** For algorithm development, data visualization, and analysis.
- **UR5 Robot Arm:** A versatile robotic arm used for demonstrating the control schemes.

## Methodologies

### Inverse Kinematics

The IK algorithm was tasked with positioning the start and end pose of the robot and then drawing three line segments to connect them. It allows for control over how the shape is drawn by adjusting the length of the parallel line segments and the number of steps per line.

![Figure 1: Robot beginning the first line](data:image/png;base64,<Base64>)

![Figure 2: Robot starting the second line](data:image/png;base64,<Base64>)

### Resolved-Rate Control

This function uses forward kinematics and the body Jacobian to control the robot's motion from a start pose to a goal pose, moving through 3 consecutive perpendicular lines.

![Figure 3: Robot approaching the end position](data:image/png;base64,<Base64>)

### Jacobian Transpose

Similar to resolved-rate control but uses the transpose of the body jacobian instead of its inverse, aiming to move the robot through a straight line from start to goal pose.

![Figure 4: Robot at the end position](data:image/png;base64,<Base64>)

### Extra Credit: Batman Symbol Drawing

Utilizing intricate mathematical equations, we plotted specific points to render the Batman symbol with the UR5 robot arm.

![Batman Symbol Drawing](data:image/png;base64,<Base64>)

## Code Structure

The project's codebase is structured into several MATLAB scripts and functions, each serving a unique purpose in the demonstration of the control schemes or the drawing of the Batman symbol.

- `Batman_equations.m`: Calculates points for the Batman symbol.
- `batman_points.m`: Generates a set of coordinates for the Batman symbol.
- `batman_real_time_plot.m`: Simulates the drawing of the Batman symbol.

## Conclusion

This project successfully demonstrates the application of various control schemes to perform precise tasks with the UR5 robot arm. Through the methodologies implemented, we have showcased the robot's ability to execute complex drawing tasks, highlighted by the creation of the Batman symbol.


