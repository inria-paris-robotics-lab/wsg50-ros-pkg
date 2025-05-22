# wsg_50_simulation 

The `wsg_50_simulation` package provides launch files to simulate the WSG 50 gripper in Gazebo along with its controllers.

This package is part of the PRL (Paris Robotics Lab) ecosystem and is designed to simplify the simulation and control of the WSG 50 gripper in Gazebo.

## Contents

### Launch Files
- **`view_wsg50.launch.py`**  
    Launches a visualization of the WSG 50 gripper.  
- **`wsg_50_controllers.launch.py`**  
    Launches the controllers for the WSG 50 gripper.  
- **`wsg_50.launch.py`**  
    Combines the simulation and controllers for the WSG 50 gripper in Gazebo.  

Refer to the documentation in the launch file headers for detailed usage instructions.

## Usage

To launch the simulation of the WSG 50 gripper in Gazebo, use the following commands:

#### Visualize the Gripper
```bash
ros2 launch wsg_50_simulation view_wsg50.launch.py
```

#### Simulate the Gripper in Gazebo with Controllers
```bash
ros2 launch wsg_50_simulation wsg_50.launch.py
```