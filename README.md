# IntelligentTrashCan

A robot trash can that comes to you.
It wakes up every minute and parks in whichever room its owner is in, and when called it drives right next to them so they can throw trash in without getting up.
It is the first robot of a larger home-cleaning automation project, built in simulation first with ROS 2 Jazzy and Gazebo Harmonic.

This repository currently holds the foundation: the simulated robot, a launch file that brings it up in Gazebo, and an automated end-to-end test.
Navigation, the house world (from [home-robot-common](https://github.com/tchausse/home-robot-common)), the simulated person and the wake/call behaviors come later.

## The robot

- **Base**: a round, low differential-drive base, 40 cm across (49 cm at the wheels), with two 17 cm driven wheels and front and rear ball casters.
  The wheel size and 6 cm of ground clearance are a starting point for the 5 cm (2 inch) bathroom step.
  Climbing it will take its own caster and drive work in a later iteration; the current base cannot.
- **Bin**: an open-top bin of about 33 liters with its rim at 62 cm, at hand height for someone sitting down.
  There is no lid, so nothing has to open.
- **Sensors**:
  - a 360 degree LiDAR on the base deck, under the bin;
  - a forward-facing RGB-D camera on the front of the bin;
  - an IMU in the base;
  - a fill-level sensor: a time-of-flight range sensor under the rim, aimed at the bin floor, that reports how full the bin is.

The workspace has three packages:

| Package | Contents |
| --- | --- |
| `trashcan_description` | xacro model, sensors, `ros2_control` setup and controller configuration |
| `trashcan_sensors` | `fill_level_node`, which turns the fill-level sensor readings into a fill percentage and a "bin full" flag |
| `trashcan_bringup` | launch file, Gazebo world, sensor bridge configuration, RViz configuration and the end-to-end test |

## Prerequisites

- Ubuntu 24.04
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), with `python3-colcon-common-extensions` and `python3-rosdep`
- Gazebo Harmonic, which `rosdep` installs through the `ros_gz` packages

Install the dependencies of the workspace from its root:

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # only once per machine
rosdep update
rosdep install --from-paths src --ignore-src -y
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch trashcan_bringup sim.launch.py
```

This starts Gazebo with its GUI, spawns the robot in a small walled room, starts the drive controllers and the fill-level monitor, and bridges the sensors to ROS.
Close the Gazebo window or press Ctrl+C to stop everything.

Launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `gui` | `true` | Show the Gazebo GUI; `false` runs headless |
| `rviz` | `false` | Also start RViz with the robot, LiDAR, depth cloud and camera image |
| `world` | `minimal_room.sdf` | Path of the Gazebo world to load |

For example, headless with RViz:

```bash
ros2 launch trashcan_bringup sim.launch.py gui:=false rviz:=true
```

### Topics

| Topic | Type | Content |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity command |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry, also published as the `odom` to `base_footprint` transform |
| `/scan` | `sensor_msgs/LaserScan` | 360 degree LiDAR |
| `/imu` | `sensor_msgs/Imu` | IMU |
| `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | RGB-D camera, in `camera_optical_frame` |
| `/bin/fill_level` | `std_msgs/Float32` | How full the bin is, from 0 (empty) to 100 percent |
| `/bin/full` | `std_msgs/Bool` | Latched; true once the bin reaches 90 percent, until it drops below 85 percent |
| `/bin/fill_range` | `sensor_msgs/Range` | Distance from the fill-level sensor to the top of the trash |

### Parameters

`fill_level_node` takes these parameters, set for the simulation in `trashcan_bringup/config/fill_level.yaml`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `empty_range_m` | required | Range in metres from the sensor to the bin floor of an empty bin |
| `full_range_m` | required | Range in metres from the sensor to the trash surface of a 100 percent full bin |
| `full_threshold_percent` | `90` | Fill percentage at which `/bin/full` turns true |
| `full_hysteresis_percent` | `5` | Percentage the fill level must drop below the threshold to clear `/bin/full` |

## Teleoperate

In a second terminal, drive the robot with the keyboard:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true
```

`stamped:=true` is needed because the drive controller takes `TwistStamped` commands.
`use_sim_time:=true` stamps those commands with simulation time, the clock the drive controller uses to stop the robot when commands stop arriving for 0.5 s, for example when teleop is closed.
Without it the commands carry wall-clock stamps, they never look stale, and the robot keeps driving.
If `teleop_twist_keyboard` is missing, install it with `sudo apt install ros-jazzy-teleop-twist-keyboard`.

## Test

```bash
colcon test
colcon test-result --verbose
```

Besides the linters and unit tests, `trashcan_bringup` runs the whole simulation headless.
It drives the robot forward for 2 simulated seconds and checks that the odometry moved more than 0.3 m, that the LiDAR sees the room walls and not the robot itself, and that the fill-level sensor reads an empty bin and then a filling one once a box is dropped in.
It uses its own Gazebo partition, so it does not interfere with a simulation you already have running.

C++ code follows the Google C++ style, enforced with the repository's `.clang-format`.
Format it with `clang-format -i $(git ls-files '*.cpp' '*.hpp')`.

GitHub Actions checks the C++ formatting and runs the same build and tests on every pull request, on a runner without a GPU, and fails the build on any compiler or build warning.

## License

[MIT](LICENSE)
