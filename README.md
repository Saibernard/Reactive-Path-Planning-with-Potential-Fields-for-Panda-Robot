# Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot
This repository contains my solution for implementing a potential field planner for the Panda robot. Using attractive and repulsive forces, the planner navigates the robot reactively in various environments, tested both in simulation and on the physical robot.


# Robot Path Planning Using Potential Fields

## Overview

I've successfully completed a project focused on robot path planning using potential fields for the Panda robot. The repository houses my implementation of a potential field-based planner, including testing in both simulation and hardware environments.

![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/6ef30bee-78d4-4adc-bdf7-d0919822c683)

![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/42cca81a-e2a6-41ea-9b41-1e60e49d48fb)


## Coding Implementation

The primary goal of this project was to develop a potential field planner for guiding the Panda robot's arm from an initial configuration to a desired goal configuration. The core implementation resides in the `potentialFieldPlanner.py` file. I meticulously designed and incorporated essential helper functions within this file to compute forces, torques, distances, and gradients required for the planning process.

## Testing and Simulation

To ensure the effectiveness of the potential field planner, I conducted extensive testing using various scenarios and environments. I experimented with different parameter values to gauge their impact on planning success rates, execution times, and the consistency of generated paths. The project includes scripts that facilitate standalone testing as well as testing within the Gazebo simulator.

![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/1e055d2e-3753-44c4-9a97-6ab9dbc58633)


![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/c8cb103b-a8a7-4242-9e0d-7d9a318280d6)

### Start position of the robot in an obstacle environment:

![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/08345c2c-d151-4ebe-a5f1-0508aae2d0fe)

### Goal position of the robot in an obstacle environment:

![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/61532fcc-898c-4c7e-93dd-7c13976258fd)

### Goal position of the robot in an obstacle environment with different start position:

![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/7afd3ca1-d6bd-4fdd-ba42-0799221b940e)

### Hardware implementation:

The robot was able to traverse the imaginary obstacle and reach the goal position:

![image](https://github.com/Saibernard/Reactive-Path-Planning-with-Potential-Fields-for-Panda-Robot/assets/112599512/359974ef-2c38-4b17-8a2b-05166a47869a)


## Usage

Utilizing my implementation involves the following steps:

1. Set up the required ROS simulation environment, configuring it for the Panda robot.
2. Navigate to the `lib` directory and execute `python potentialFieldPlanner.py` to test the planner's functionality in a controlled setting.
3. For a more comprehensive assessment, employ Gazebo to simulate the robot's movements. Launch the simulator using relevant maps and test scripts.

## Conclusion

The successful completion of this project has equipped me with a deep understanding of potential field-based planners for robot path planning. The inherent reactivity of potential fields in unknown environments makes this approach suitable for various real-world robotic applications. Throughout the implementation and testing phases, I gained valuable insights into parameter tuning and comprehending the trade-offs between potential fields and conventional planners such as A* and RRT. This repository stands as a valuable resource for anyone interested in exploring potential field-based strategies for robotic path planning.
