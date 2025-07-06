# Multi-Robot TurtleBot 4 ROS 2 Demos (Jazzy)

This repository contains a suite of autonomous navigation and multi-robot coordination demos developed using **TurtleBot 4 Lite** and **ROS 2 Jazzy**. The work showcases scripting-based robot behaviors like motion patterns, pose resets, following logic, and SLAM map generation.

---

## Key Features

-  **Square Path Motion**
-  **Zigzag Path Motion**
-  **Move to Predefined Goal Coordinates** (e.g., (5, 5))
-  **Reset to Origin** (0, 0, θ = 0)
-  **Robot Following Using Odometry**
-  **Two-Robot Formation Behavior**
-  **SLAM Map Generation using `slam_toolbox`**

> All behaviors are implemented as individual Python ROS 2 packages using `rclpy`.

---

##  Repository Structure (Simplified)

```

Multi-Robot-demo/
├── MAP using Slam/ # Saved maps (.pgm, .yaml)
├── Media/ # Demo GIFs (square.gif, follower.gif)
|
src/
├── square_movement/          # Square path script
├── zigzag_movement/          # Zigzag motion logic
├── go_to_point/              # Move to (x, y)
├── reset_pose/               # Return to origin
├── robot_follower/           # One robot follows another
├── robot_formation/          # Two robots move in formation
├── mimic/                    # Behavior mimicry (e.g., replay motion)


````

---
-----

## Running the Demos

To get the demos up and running in this repository:

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/VERYsmallBOI/Multi-Robot-demo.git
    ```
    
2.  **Navigate into the workspace:**

    ```bash
    cd Multi-Robot-demo
    ```

3.  **Build and source your workspace:**

    ```bash
    colcon build
    source install/setup.bash
    ```

    
4.  **Run a demo:**
    For example, to run the square movement demo for `robot2`:

    ```bash
    ros2 run square_movement square_driver.py
    ```

 **Tip**: Ensure all scripts are executable and ROS dependencies installed (`rclpy`, `geometry_msgs`,`tf-transformations`, etc.).

---

## SLAM and Mapping

This repo includes SLAM-generated maps using `slam_toolbox`. To launch the generated Map:

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=atlantis.yaml
ros2 launch turtlebot4_navigation nav2.launch.py
```
### Sample SLAM Map

<p align="center">
  <img src="MAP%20using%20Slam/Atlantis.png" alt="SLAM Map">
</p>

---

##  Demo Previews

###  Square Path

<p align="center">
  <img src="Media/square.gif" alt="Square Path" width="400">
</p>

###  Robot Following

<p align="center">
  <img src="Media/follower.gif" alt="Follower" width="400">
</p>


---


## Requirements
* ROS 2 Jazzy (Ubuntu 24.04)
* TurtleBot 4 Lite (Create 3 + Raspberry Pi)
* Python 3.12
* ROS 2 packages:

  * `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf_transformations`

Install additional Python dependencies if needed like this(doesn't affect the system-wide python) :

```bash
sudo apt update && sudo apt install ros-jazzy-tf-transformations
```

---


## Maintainer

**Tamil Selvan**
 [tamilselvanelango3@gmail.com](mailto:tamilselvanelango3@gmail.com)



---
