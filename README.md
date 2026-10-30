# Autonomous Mobile Robot (ROS 2) – 4-wheel skid-steer

ESP32 motor controller + Raspberry Pi/PC running ROS 2. Current state: **wheel odometry +
IMU fusion**. SLAM and Nav2 are not part of the repo yet.

## Packages (`src/`)

| Package | Purpose |
|---|---|
| `amr_description` | URDF/Xacro, meshes, Gazebo world, RViz config |
| `amr_controller` | `diff_drive_controller` config, joystick teleop, optional noisy-odometry node |
| `amr_firmware_esp32` | ros2_control hardware interface (serial to the ESP32), MPU6050 driver, ESP32 firmware (`firmware/4wheel_robot_control`) |
| `amr_localization` | `robot_localization` EKF config/launch, localization experiments |
| `amr_msgs` | custom services |
| `amr_launch` | top-level `simulation.launch.py` and `real_robot.launch.py` |

## Dependencies

Replace `$ROS_DISTRO` with your distro (the Gazebo launch picks Ignition on Humble, gz otherwise).

```bash
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joy-teleop \
  ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-ros-gz libserial-dev python3-smbus
# plus ign_ros2_control (Humble) or gz_ros2_control (newer) for simulation
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch amr_launch simulation.launch.py                      # Gazebo
ros2 launch amr_launch real_robot.launch.py port:=/dev/ttyUSB0   # real robot
```

Optional experiment – fuse deliberately noisy odometry (simulation only):

```bash
ros2 launch amr_launch simulation.launch.py use_noisy_odom:=true
ros2 launch amr_localization local_localization.launch.py use_sim_time:=true use_noisy_odom:=true
```

## Data flow and TF

```
ESP32 <--serial--> AmrInterface (ros2_control) --> amr_controller (diff_drive) --> /amr_controller/odom
MPU6050 --I2C--> mpu6050_driver --> /imu/out (frame imu_link) ----------------+
                                                                              v
                                      ekf_filter_node (vx from odom + yaw-rate from gyro) --> /odometry/filtered
```

TF chain: `odom -> base_footprint -> base_link -> {wheels, imu_link}`.
Exactly **one** node may publish `odom -> base_footprint`:

* **Default:** `amr_controller` (`enable_odom_tf: true`), the EKF only publishes `/odometry/filtered`.
* **EKF owns odom** (what you will want for Nav2): set `enable_odom_tf: false` in
  `amr_controller/config/amr_controllers.yaml` **and** launch the EKF with `ekf_publishes_tf:=true`.

## Calibration

* **Gyro bias** – automatic: `mpu6050_driver` averages 200 samples at start-up. Keep the robot
  still for the first ~2 s. Disable with `-p gyro_calibration_samples:=0`.
* **IMU orientation sanity check** – turn the robot counter-clockwise by hand:
  `ros2 topic echo /imu/out --field angular_velocity.z` must go **positive**. If not, the
  `imu_joint` in the URDF does not match how the board is mounted.
* **`wheel_separation_multiplier`** (skid-steer slip) – spin the robot in place for several full
  turns and compare odometry yaw with the real rotation:
  `new = old * (yaw_reported_by_odom / real_yaw)`. Repeat until they agree.
* **Wheel radius** – drive 2–3 m straight, compare odometry distance with a tape measure,
  scale `wheel_radius` accordingly.

## Safety

* ESP32 firmware stops all wheels if no command arrives for 500 ms.
* `AmrInterface` sends a zero command when deactivated.
* Still add a physical power switch/e-stop for the motor supply.

## Status / TODO

- [ ] Lidar in URDF + driver
- [ ] SLAM Toolbox, Nav2 (remember: Nav2 on Humble publishes `Twist`, the controller expects
      `TwistStamped` on `amr_controller/cmd_vel` -> needs a converter or `twist_mux`)
- [ ] IMU sensor for the Gazebo model (frame must be `imu_link`)
