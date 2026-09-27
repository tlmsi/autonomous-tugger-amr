# Autonomous Tugger AMR

A ROS 2 and Gazebo Sim project focused on developing an **autonomous industrial tugger AMR** capable of navigation, trailer docking, attachment, material transport, and delivery.

The project is being developed incrementally, beginning with the mobile robot platform and progressing toward autonomous navigation, precision docking, trailer handling, and mission-level control.

> **Status:** Work in progress.

---

## Project Objective

The final system is intended to perform an autonomous material transport mission:

1. Receive a transport mission
2. Navigate to the trailer pickup area
3. Approach a pre-docking pose
4. Perform precise trailer alignment and docking
5. Attach the trailer
6. Adapt navigation behavior for trailer towing
7. Transport the trailer through the industrial environment
8. Dock at the delivery station
9. Detach the trailer
10. Complete the mission or return home

Recovery and safety behavior will also be introduced for docking failures, blocked paths, attachment failures, and unsafe trailer articulation.

---

## Current Progress

The project is currently in the **AMR platform development phase**.

Implemented so far:

- ROS 2 workspace and package structure
- Custom URDF/Xacro robot model
- Four-wheel-steering vehicle architecture
- Four steering joints
- Four wheel joints
- Rear-wheel-drive concept
- Custom low-profile AMR body geometry
- Front and rear camera housings
- Two diagonally positioned LiDAR housings
- IMU frame
- Rear hitch frame
- RViz visualization
- Initial Gazebo Sim package and test world

---

## Vehicle Architecture

The AMR uses a **four-wheel-steering (4WS)** architecture.

The physical vehicle concept consists of four main actuators:

- Front steering actuator
- Rear steering actuator
- Rear-left traction motor
- Rear-right traction motor

The URDF represents each wheel steering angle individually so that the vehicle controller can later calculate the required steering geometry for all four wheels.

The planned control architecture is:

```text
Velocity Command
       |
       v
4WS Kinematic Controller
       |
       +--------------------+
       |                    |
       v                    v
Steering Geometry      Traction Control
       |                    |
 FL FR RL RR           RL / RR Velocity
