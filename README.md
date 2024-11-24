# Four Drones Repel System

This repository contains a project that involves simulating the interaction and repulsion between four drones, utilizing ROS (Robot Operating System), Gazebo simulation, and linear MPC (Model Predictive Control). The objective is to enable the drones to maintain a safe distance from one another in both middle and inner regions by controlling their velocities and publishing appropriate repelling waypoints. This README file will help you understand the structure and functionality of the code files used in this project.

## Overview

The goal of the project is to simulate the behavior of four drones that actively repel each other if they come too close, using ROS to manage drone positions and velocities. Each drone maintains a safe distance from others by calculating repulsion forces, updating target waypoints accordingly, and repelling based on position and velocity data.

The key components of this project include:

1. **Drone Repel Node**: A C++ script to calculate the relative distances between drones, calculate repelling forces, and update drone positions accordingly.
2. **Gazebo Simulation Setup**: A launch file that initializes the drones in the Gazebo simulation environment.
3. **Observer and Controllers**: Integration with MAV controllers and Gazebo plugins to manage drone behavior.

### Requirements

- ROS Melodic or later
- Gazebo Simulator (Version compatible with your ROS version)
- MAVROS packages
- C++14 compatible compiler

To install the necessary ROS packages and MAVROS, you can use:

```sh
sudo apt-get install ros-<your_ros_version>-mavros ros-<your_ros_version>-mavros-extras
```

Replace `<your_ros_version>` with the version of ROS you are using.

### Repository Structure

```
- config/
    - params.yaml          # Parameters file for drone settings
- launch/
    - four_drones.launch   # Main launch file to start Gazebo with four drones
- src/
    - drone_repel_node.cpp # Main C++ node that controls drone repelling behavior
    - app_vel.cpp          # Additional script to compute velocity adjustments
- models/
    - urdf/
        - mav_odometry_and_camera.gazebo # URDF model for drones with odometry and camera
- resources/
    - linear_mpc_*.yaml    # Configuration files for MPC controller
    - disturbance_observer_*.yaml  # Configuration for disturbance observer
```

## Launching the Simulation

To start the simulation environment, use the following command:

```sh
roslaunch four_drones four_drones.launch
```

This command launches the Gazebo world and spawns four drones. Each drone is given an initial position and starts executing a waypoint mission, using a PID attitude controller and a linear MPC.

## Key Code Components

### drone_repel_node.cpp

The `drone_repel_node.cpp` script handles the main logic of the drone repelling system:

- **Subscribers**: The node subscribes to position and velocity topics for all drones (`/fireflyX/ground_truth/position`, `/fireflyX/mav_linear_mpc/KF_observer/observer_state`), to receive continuous updates of their states.
- **Callbacks**: Callback functions (`subcallback1`, `sub_odom_callback1`, etc.) store data from these topics.
- **Calculations**:
  - The function `app_vel` calculates the approach velocity between drones, which is used to decide if a drone is approaching dangerously close to another drone.
  - Based on distances (`r12`, `r13`, etc.) and relative velocities, new target positions are calculated and published for each drone to avoid collisions.
- **Publishers**: Updated positions for each drone are published to `command/pose` topics to adjust the drones' movements.

### four_drones.launch

The `four_drones.launch` file is responsible for launching the simulation with four drones in Gazebo.

- **World Initialization**: It loads the Gazebo world and required models.
- **Drone Spawning**: Four separate instances of the drone model are spawned in Gazebo, each assigned with a namespace (`firefly1`, `firefly2`, etc.). The drones are initialized with different starting coordinates.
- **MPC and PID Controllers**: Controllers are defined for each drone to ensure stable flight and movement.

### mav_linear_mpc

The `mav_linear_mpc` is used for controlling the flight path of the drones. The configuration files (`linear_mpc_*.yaml`) define the parameters for the MPC to ensure accurate movement.

### Simulation Workflow

1. **Gazebo World Initialization**: The `four_drones.launch` file starts Gazebo, loads the world, and spawns four drones.
2. **Physics Activation**: A call is made to unpause the physics in Gazebo, ensuring that the environment and drones can interact.
3. **State Publishing and Adjustment**: Each drone's current position and velocity are obtained through ROS subscribers, and based on the calculations, the repelling algorithm is executed.
4. **Waypoint Update**: The calculated new positions are published to each drone's `command/pose` topic, ensuring that drones maintain safe distances from each other.

### Gazebo Models

The `mav_odometry_and_camera.gazebo` model provides odometry sensors and a camera for each drone. This allows the drones to sense their environment and simulate realistic autonomous behavior.

## Parameters and Configuration

- **Repelling Constants**: The `drone_repel_node.cpp` script uses various parameters for determining the strength of repulsion (`repel_const`, `vel_repel_const`, `exp_r`, etc.). These parameters are configurable in the `params.yaml` file.
- **Minimum Safe Distance**: `min_dist` defines the minimum allowable distance between drones. If drones get closer than this distance, the repelling forces are activated.
- **Control Rates**: The node operates at a rate defined by `rate`, which determines the frequency of drone position and velocity updates.

## Usage and Visualization

- **RViz and Gazebo**: Use Gazebo to visualize the drone simulation. RViz can be used for additional data visualization if needed.
- **ROS Topics**: The drones publish their current states on topics like `/fireflyX/ground_truth/position`. Use `rostopic echo` to monitor these topics during simulation:

  ```sh
  rostopic echo /firefly1/ground_truth/position
  ```

## Troubleshooting

- **Gazebo Fails to Unpause**: If Gazebo does not unpause, make sure that the simulation environment has been properly set up with all necessary packages.
- **Controller Errors**: Ensure the `mav_linear_mpc` and PID controllers are configured properly in their respective YAML files.
- **Position Updates Not Working**: Check the topics to verify if the callbacks are being triggered correctly. Verify that the `ros::spin()` and `ros::spinOnce()` functions are properly used to handle callbacks.

## Future Improvements

- **Dynamic Obstacle Avoidance**: The current implementation only considers other drones. Integration with dynamic obstacles (like humans or cars) could improve simulation.
- **Complex Formations**: Implement a complex flight formation for the drones that maintains a pattern while avoiding collisions.
- **GUI for Parameters**: Adding a user interface to adjust parameters like repulsion constant, minimum distance, and velocity constants could make tuning easier.

Feel free to raise issues or contribute to the project by creating a pull request.
