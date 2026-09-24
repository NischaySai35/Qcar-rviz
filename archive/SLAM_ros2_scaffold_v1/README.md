# QCar 2 SLAM & Nav2 Autonomous Navigation

This package provides a Python-first implementation for SLAM and autonomous navigation for the Quanser QCar 2 using ROS2 Humble and Nav2.

## Workspace Location
**Note:** Due to environment restrictions, the code has been generated in:
`/home/nvidia/.gemini/antigravity/scratch/SLAM`

## Quick Start Guide

### 1. Build the Workspace
```bash
cd /home/nvidia/.gemini/antigravity/scratch/SLAM
colcon build --symlink-install
source install/setup.bash
```

### 2. Verify Sensors (LiDAR, Odom, CSI)
Ensure your QCar 2 drivers are running in another terminal, then run:
```bash
ros2 run qcar2_nav verify_sensors
```
This script uses the **verified "best" parameters** for your CSI cameras (820x616 @ 80fps) and checks for active LiDAR and Odometry topics.

### 3. Launch SLAM and Nav2
If the verifier says "READY", run:
```bash
ros2 launch qcar2_nav navigation.launch.py
```
This will launch:
- `slam_toolbox` (Asynchronous Mapping)
- `nav2_bringup` (Navigation stack with Smac Planner and Regulated Pure Pursuit)
- `rviz2` (Visualization)

### 3. Run Python Navigation Scripts
In a new terminal (don't forget to source):
**Send a Single Goal:**
```bash
ros2 run qcar2_nav send_goal
```
**Follow Multiple Waypoints:**
```bash
ros2 run qcar2_nav follow_waypoints
```

## RViz Setup & Topics
In RViz, please add the following displays to verify the system:
- **Map:** Topic `/map` (Verify the 2D lab hall is being built).
- **Robot Model:** Verify `base_link` is correctly localized.
- **Laser Scan:** Topic `/scan` (Ensure alignment with the map).
- **Global Path:** Topic `/plan` (Verify the shortest path).
- **Local Plan:** Topic `/local_plan` (Verify RPP controller behavior).
- **Costmaps:** `/local_costmap/costmap` and `/global_costmap/costmap` (Check inflation).

## Parameter Recommendations
- **Inflation Radius:** Set to `0.55m` in `nav2_params.yaml` to ensure the car (approx 0.4m long) has enough clearance from walls/obstacles.
- **Controller:** `RegulatedPurePursuitController` is used as it is better suited for the Ackermann-like steering of the QCar 2 than standard DWA.
- **Obstacle Layer:** Configured to raytrace up to 3m and mark obstacles up to 2.5m.

## Troubleshooting Checklist
- **TF Tree:** Run `ros2 run tf2_tools view_frames` and ensure `map -> odom -> base_link` is connected.
- **Odom Drift:** If the map rotates wildly, check your wheel encoders and IMU fusion.
- **Laser Alignment:** If the scan doesn't match the map walls, check the precision of your `base_link -> laser` transform.
- **Costmap Issues:** If the car refuses to move, check for "ghost" obstacles in the local costmap. Use `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}` if needed.
- **Nav2 Not Active:** If scripts hang at `waitUntilNav2Active()`, ensure all Nav2 nodes in `navigation.launch.py` started successfully and are in the `ACTIVE` lifecycle state.

## Rerouting & Obstacle Avoidance
The `RegulatedPurePursuitController` is configured with `use_collision_detection: True`. If a dynamic obstacle appears on the path:
1. The **Local Costmap** will mark the obstacle.
2. The **Controller** will attempt to steer around if space is available.
3. If the path is completely blocked, the **BtNavigator** will trigger a global replan or recovery behavior.
