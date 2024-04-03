
# UR5 Robot "Place-and-Draw" Task Project
 ![Seyi's%20Video%20-%20Apr%203%2C%202024](https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/c90ffc7d-1856-41c0-b7ab-4463ea19785c)
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
<p align="center">
  <img width="320" alt="Screenshot 2024-04-03 at 4 38 20 PM" src="https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/95cb3193-2573-4cf3-bfb5-dd9a08080a45"> 
  <img width="360" alt="Screenshot 2024-04-03 at 4 41 30 PM" src="https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/9834414b-dae7-42da-ab40-20542fb8f776">
</p>



### Resolved-Rate Control

The resolve-rate function uses forward kinematics and the body Jacobian to control the robot's motion from a start pose to a goal pose, moving through 3 consecutive perpendicular lines.

### Jacobian Transpose

Similar to resolved-rate control but uses the transpose of the body jacobian instead of its inverse, aiming to move the robot through a straight line from start to goal pose.

### Extra Credit: Batman Symbol Drawing

Utilizing intricate mathematical equations, I plotted specific points to render the Batman symbol with the UR5 robot arm to help my team get some extra credit.
<p>
  <img width="481" alt="Screenshot 2024-04-03 at 4 56 15 PM" src="https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/2bd0c95d-0571-49d4-9c3f-81174033d52f" style="margin-right: 10px;">
  <img width="428" alt="Screenshot 2024-04-03 at 4 56 48 PM" src="https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/27b36d31-3653-4056-a828-0f023ad41cec">
</p>


## Code Structure

The project's codebase is structured into several MATLAB scripts and functions, each serving a unique purpose in the demonstration of the control schemes or the drawing of the Batman symbol.

- `Batman_equations.m`: Calculates points for the Batman symbol.
- `batman_points.m`: Generates a set of coordinates for the Batman symbol.
- `batman_real_time_plot.m`: Simulates the drawing of the Batman symbol.

## Conclusion

This project successfully demonstrates the application of various control schemes to perform precise tasks with the UR5 robot arm. Through the methodologies implemented, we have showcased the robot's ability to execute complex drawing tasks, highlighted by the creation of the Batman symbol.

