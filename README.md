# imu_fusion_ros

A **ROS 2 (Humble)** package for fusing IMU and magnetometer data into a stable orientation, using the [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) library.  

## Features

1. **Subscribes** to:
   - `/imu/data_raw` (sensor_msgs/Imu)
   - `/imu/mag` (sensor_msgs/MagneticField)

2. **Publishes** (if enabled via parameters):
   - `/imu/data` (sensor_msgs/Imu) – fused orientation
   - TF transform from `map_frame` to `target_frame`.

3. **Service** to trigger calibrations:
   - `trigger_calibration` (from `imu_fusion_ros_interface/CalibrationTrigger` srv).
     - Accepts a string such as `"magnetometer"`, `"accelerometer"`, `"gyro"`, `"heading"`, etc.

## Parameters

- `publish_fused_imu` (bool, default: `true`): Whether to publish the fused IMU data.
- `publish_tf` (bool, default: `true`): Whether to broadcast a TF transform.
- `map_frame_id` (string, default: `"map_frame"`): The frame ID for the "map" or reference frame.
- `target_frame_id` (string, default: `"base_link"`): The child frame ID, e.g., your robot base link.

## Usage

1. **Install** or **Clone** the the library:
   ```bash
   cd your_ros2_ws/src
   git clone --recursive https://github.com/TPODAvia/imu_fusion_ros.git
   ```

2. **Build** the workspace:

   ```bash
   cd your_ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Run** the launch file:

   ```bash
   ros2 launch imu_fusion_ros imu_fusion.launch.py
   ```

4. **Trigger calibrations** (example):

   ```bash
   ros2 service call /trigger_calibration imu_fusion_ros_interface/srv/CalibrationTrigger "{calibration_type: 'magnetometer'}"
   ```

4. **Run in Docker**:

   Still in the development

## Dependencies

- ROS 2 Humble
- [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion)
- `imu_fusion_ros_interface` (for the calibration service definition)

## License

Apache-2.0 (or whichever license you prefer).

---

**Maintainer**: [Your Name](mailto:you@yourdomain.tld)
```

