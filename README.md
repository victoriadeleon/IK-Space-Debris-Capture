# Simulated Robotic Capture of Space Debris Using Inverse Kinematics Control
Victoria de León | Tokyo Metropolitan University | June 2025

### Overview
Space debris represents a critical challenge to current and future space operations, with thousands of uncontrolled objects threatening satellites and crewed missions. This project presents a simulated robotic capture system developed using CoppeliaSim Edu 4.7.0, integrating inverse kinematics, vision-based detection, proximity sensing, and Kalman filtering to autonomously intercept and grasp free-floating debris.

This work was developed as part of a research stay at the laboratory of Dr. Hirohisa Kojima at Tokyo Metropolitan University, Tokyo, Japan, focused on advancing autonomous systems for space debris removal and on-orbit servicing.

### Key Features
1. Robotic Configuration and Simulation Environment
A UR5 robotic manipulator and a BarrettHand gripper were configured in CoppeliaSim.

The simulation models realistic microgravity conditions, with space debris exhibiting floating motion, varying velocities, and orientations.

Debris models were designed in Blender and imported with correct physical scaling for simulation accuracy.

The reachable workspace of the UR5 was implemented based on manufacturer specifications to prevent invalid or unreachable arm configurations.

2. Vision-Based Zone Detection
A vision sensor mounted on the gripper continuously analyzes the vertical location of the target object.

The image is divided into three vertical regions (top, middle, bottom), and the brightest region is interpreted as the dominant zone.

This zone is encoded as a signal used to control vertical adjustments of the robotic arm in real time.

3. Inverse Kinematics and Adaptive Positioning
A custom inverse kinematics setup calculates real-time target alignment using the simIK module.

The target point dynamically shifts based on the vision sensor signal, allowing the end-effector to align with the debris as it moves.

Fine Z-axis adjustments are executed smoothly, ensuring that the capture occurs within valid motion bounds.

4. Proximity-Based Capture Activation
A proximity sensor attached to the gripper detects when the object is within a predefined capture range.

Upon detection, the object is dynamically attached to the gripper at a set offset and orientation.

A float signal is used to actuate the BarrettHand fingers, triggering closure only upon valid capture conditions.

5. Kalman Filter for Motion Prediction
A Kalman filter tracks and estimates the Z-axis position and velocity of the debris in real time.

Based on this estimate, the system predicts the time remaining before the object reaches the optimal capture zone.

This prediction is logged and can be used to further optimize the decision-making process for grasp execution.

Demonstration Videos
Demo 1: Vision-guided capture using Kalman prediction

Demo 2: Proximity-based satellite interception

Demo 3: Workspace-aware adaptive IK alignment



![image](https://github.com/user-attachments/assets/e0f0e02a-5988-46e6-9e0f-fe7ebe08aeea)
