# ratslam

This package is a port of the original [ratslam_ros](https://openslam-org.github.io/openratslam.html) repository to ROS 2 (tested on ROS 2 Rolling).

## Dependencies

In addition to standard ROS 2 dependencies, this package **requires the custom `topological_msgs` package**. Make sure it is available and built in your workspace before building `ratslam`.

Main dependencies:
- `rclcpp`
- `topological_msgs` (custom)
- `cv_bridge`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `visualization_msgs`
- `tf2_ros`
- `image_transport`
- `Boost` (component `serialization`)
- `OpenCV`
- `Irrlicht`
- `OpenGL`

## Installation


1. **Clone the `ratslam` and `topological_msgs` repositories into your workspace:**

   ```bash
   cd ~/rolling_ws/src
   git clone https://github.com/BorgesJVT/ratslam.git
   git clone https://github.com/BorgesJVT/topological_msgs.git
   ```

2. **Install system dependencies:**

   ```bash
   sudo apt update
   sudo apt install ros-rolling-cv-bridge ros-rolling-sensor-msgs ros-rolling-geometry-msgs ros-rolling-nav-msgs ros-rolling-visualization-msgs ros-rolling-tf2-ros ros-rolling-image-transport libboost-all-dev libopencv-dev libirrlicht-dev libopengl-dev
   ```

3. **Build the workspace:**

   ```bash
   cd ~/rolling_ws
   source /opt/ros/rolling/setup.bash
   colcon build --symlink-install
   ```

4. **Source the workspace:**

   ```bash
   source ~/rolling_ws/install/setup.bash
   ```

## Usage
To use this package with one of the example datasets, first convert the dataset to a ROS 2 bag format if necessary.

```bash
ros2 bag play data/irat_aus_28112011/irat_aus_28112011.db3 --rate 1.0 --clock --start-paused --topics /irat_red/odom /irat_red/camera/image/compressed
```

In another terminal, run one of the ratslam nodes, for example:

```bash
ros2 launch ratslam irataus.launch
```