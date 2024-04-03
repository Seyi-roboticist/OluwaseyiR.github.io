
# UR5 Robot EST Path Planner

This repository is dedicated to the development of an Expansive Space Tree (EST) path planner for the UR5 robot arm. The project integrates with the MoveIt! library to enable complex motion planning capabilities within a ROS (Robot Operating System) environment. This work was undertaken as part of the "Algorithm for Sensor-Based Robotics" course.

## Project Overview

The goal of this project is to implement a robust path planner that allows a UR5 robot arm to navigate through environments with obstacles, reaching specified target configurations without collisions. This involves creating a custom algorithm based on the principles of EST path planning, leveraging ROS and MoveIt! for simulation and visualization.

## Key Features

- **EST Path Planning**: Custom implementation designed for the UR5 robotic arm.
- **Collision Detection**: Utilizes MoveIt! for accurate collision avoidance with environmental obstacles.
- **Dynamic Obstacle Integration**: Supports adding and configuring obstacles within the planning scene.
- **Goal Configuration**: Allows for the setting of specific target states for the robot.
- **Visualization and Simulation**: Integrated with RViz for real-time visualization of the robot's motion and planning environment.

## Getting Started

This section is deliberately kept generic to inspire exploration and understanding of ROS, MoveIt!, and the fundamentals of path planning without providing direct implementation details.

- **Prerequisites**: Ensure you have a working ROS environment, with necessary dependencies like MoveIt! installed.
- **Installation**: Clone this repository into your ROS workspace's `src` folder. Build the workspace using appropriate ROS build tools.
- **Running the Planner**: Use ROS launch files to start the planner. Explore various configurations and obstacles to understand the planner's capabilities.

## Contributing

Contributions are encouraged for improving the algorithm, adding new features, or enhancing the documentation. Please adhere to standard open-source contribution guidelines.

## License

This project is open-source and available under a suitable license, facilitating educational use and collaboration.

