
# UR5 Robot EST Path Planner
![ur5EST](https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/cdd1d19e-3765-42c3-9cce-d4a27d502e23)

This repository is dedicated to the development of an Expansive Space Tree (EST) path planner for the UR5 robot arm. The project integrates with the MoveIt! library to enable complex motion planning capabilities within a ROS2 (Robot Operating System 2) environment. This work was undertaken as part of the "Algorithm for Sensor-Based Robotics" (ASBR) course.

## Project Overview

The goal of this mini project (assignment 3 of the ASBR course) is to implement a robust path planner that allows a UR5 robot arm to navigate through environments with obstacles, reaching specified target configurations without collisions. This involves creating a custom algorithm based on the principles of EST path planning, leveraging ROS and MoveIt! for simulation and visualization. This was a three weeks long assignment which required a lot of intuition and lots of coding. Unfortunately, I am unable to share the code on public platforms like GitHub due to the professor's policy of reusing assignments. For instance, in subsequent classes, students may be tasked with designing and implementing various planning algorithms such as RRT, RRT*, PRM, EST, BiEST etc., utilizing the MoveIt! software.

## Key Features
Some of the key features of what I did include the following:
- **EST Path Planning**: Custom implementation designed for the UR5 robotic arm.
- **Collision Detection**: Utilizes MoveIt! for accurate collision avoidance with environmental obstacles.
- **Dynamic Obstacle Integration**: Supports adding and configuring obstacles within the planning scene.
- **Goal Configuration**: Allows for the setting of specific target states for the robot.
- **Visualization and Simulation**: Integrated with RViz for real-time visualization of the robot's motion and planning environment.

## Getting Started

This section is deliberately kept generic to inspire exploration and understanding of ROS2, MoveIt!, and the fundamentals of path planning without providing direct implementation details.

- **Prerequisites**: Ensure you have a working ROS or ROS2 environment, with necessary dependencies like MoveIt! installed.
- **Installation**: Read up on MoveIt! and install it on your machine.
- **Running the Planner**: Use ROS2 launch files to start the planner. Explore various configurations and obstacles to understand the planner's capabilities.


## License

This project is open-source and available under a suitable license, facilitating educational use and collaboration.

