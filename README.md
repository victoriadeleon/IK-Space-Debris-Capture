# Simulated Robotic Capture of Space Debris Using Inverse Kinematics Control  

**ヴィクトリア・デ・レオン**  
**東京都立大学 | Tokyo Metropolitan University**  
**June 2025**


## Overview

Space debris poses a growing threat to operational satellites and future space infrastructure. Autonomous robotic systems capable of identifying, tracking, and capturing debris are essential for long-term orbital sustainability. This project presents a simulated debris capture system using **CoppeliaSim Edu v4.7.0**, combining:

- Inverse kinematics
- Vision-based adaptive positioning
- Proximity-based grasp triggering
- Kalman filtering for motion prediction

Developed at the **Laboratory of Dr. Hirohisa Kojima**, **Tokyo Metropolitan University**, Tokyo, Japan.

## 🔧 System Description

### 🤖 Robot Configuration

- **UR5 robotic arm** with six degrees of freedom.
- **BarrettHand gripper** using sample model and custom control logic.
- **Vision sensor** mounted at the gripper tip.
- **Proximity sensor** for close-range object detection.
- Imported 3D debris models (e.g., satellites) from Blender, scaled realistically.

### 🌌 Environment Setup

- **Microgravity simulation:** No gravity and floating debris.
- **Variable velocities and orientations** applied to simulate free-floating space objects.
- **Workspace validation** using UR5 kinematic specifications to avoid invalid positions or unreachable targets.

## 🧠 Functional Components

### 1. Vision-Based Target Zoning
- The vision sensor analyzes pixel brightness in real time.
- The image is split into 3 vertical zones: `bottom`, `middle`, `top`.
- The dominant zone (highest pixel activity) is broadcast via an integer signal:  
  - `0` → bottom  
  - `1` → middle  
  - `2` → top
- This zone influences Z-axis movement for target alignment.

### 2. Inverse Kinematics (IK) for Arm Control
- The `simIK` environment calculates arm poses based on the target position.
- Z-position of the target is adaptively updated using the signal from the vision sensor.
- Arm motion respects physical workspace limits (radius ≤ 0.85 m).
- End-effector position is validated via real-time error measurement (threshold: 2 cm).

### 3. Proximity-Based Capture Trigger
- A proximity sensor is used to detect when the object is within the grasp zone.
- Upon detection:
  - The object is re-parented to the gripper’s attach point.
  - A float signal is sent to actuate the fingers.
  - Object is repositioned and reoriented for a clean grasp.

### 4. Kalman Filter for Z-Motion Prediction
- Tracks the Z-position of the debris.
- Estimates vertical velocity and predicts time until reaching the optimal capture zone.
- Prediction outputs:
  - Estimated Z position
  - Vertical speed
  - Time-to-capture (used for logging or planning)

## 📽️ Demonstration Videos

| Description | Link |
|------------|------|
| Vision-guided capture using Kalman prediction | [Watch here](https://youtu.be/GTcBJm_dImc) |
| Proximity-based satellite interception | [Watch here](https://youtu.be/qegu02heNtw) |
| Workspace-aware adaptive IK alignment | [Watch here](https://youtu.be/QFFG4ZrGdR4) |

