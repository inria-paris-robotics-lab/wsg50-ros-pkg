### Changelog

#### [1.0.0] - 2025-04-22

##### Added
- **Hardware Interface**: Implemented a hardware interface for ROS 2 Control (`WSG50HardwareInterface`).
- **Action Server**: Replaced previous ROS services with a ROS 2 action server for movement commands (`move`, `grasp`, `release`).
- **Configuration Services**: Retained services for one-time configuration tasks:
    - **Homing**: Recalibration of mechanical limits.
    - **Tare**: Resetting force measurements.
    - **Set_acc / Set_force**: Dynamic adjustment of acceleration or grasping force.
- **URDF Update**: Updated URDF description to better align simulated functionality with real-world behavior.

##### Changed
- **Command Handling**: Transitioned from ROS 1 services to ROS 2 actions for main commands.
- **Communication Modes**: Unified communication modes into a single `Auto_update` mode.

##### Removed
- **Functioning Modes**: Removed irrelevant communication modes for our usages (`Polling`, `Script`).


