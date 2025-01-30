#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// Include the service definition
#include "imu_fusion_ros_interface/srv/calibration_trigger.hpp"

// Fusion library headers
#include "Fusion/Fusion/Fusion.h"

using namespace std::chrono_literals;

#define DEFAULT_SAMPLE_RATE 100.0f

class ImuFusionNode : public rclcpp::Node
{
public:
  ImuFusionNode() : Node("imu_fusion_node")
  {
    // 1. Declare parameters (topics, frames, AHRS settings, etc.)
    imu_raw_topic_   = this->declare_parameter<std::string>("imu_raw_topic", "imu/data_raw");
    mag_topic_       = this->declare_parameter<std::string>("mag_topic", "imu/mag");
    fused_imu_topic_ = this->declare_parameter<std::string>("fused_imu_topic", "imu/data");

    publish_fused_imu_ = this->declare_parameter<bool>("publish_fused_imu", true);
    publish_tf_        = this->declare_parameter<bool>("publish_tf", true);
    use_gravity_compensated_accel_ = this->declare_parameter<bool>("use_gravity_compensated_accel", false);
    map_frame_id_      = this->declare_parameter<std::string>("map_frame_id", "map_frame");
    target_frame_id_   = this->declare_parameter<std::string>("target_frame_id", "base_link");

    // AHRS-related settings
    gain_                    = this->declare_parameter<double>("gain", 0.5);
    gyro_range_              = this->declare_parameter<double>("gyro_range", 2000.0);
    acceleration_rejection_  = this->declare_parameter<double>("acceleration_rejection", 10.0);
    magnetic_rejection_      = this->declare_parameter<double>("magnetic_rejection", 10.0);
    recovery_trigger_period_ = this->declare_parameter<int>("recovery_trigger_period", (int)(5 * DEFAULT_SAMPLE_RATE));

    // 2. Load calibration parameters from YAML
    // These are just declared here; the actual .yaml can be loaded at runtime.
    std::vector<double> gyro_misalignment = this->declare_parameter<std::vector<double>>(
      "gyroscope_misalignment",
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0}
    );
    std::vector<double> gyro_sensitivity = this->declare_parameter<std::vector<double>>(
      "gyroscope_sensitivity",
      {1.0, 1.0, 1.0}
    );
    std::vector<double> gyro_offset = this->declare_parameter<std::vector<double>>(
      "gyroscope_offset",
      {0.0, 0.0, 0.0}
    );

    std::vector<double> accel_misalignment = this->declare_parameter<std::vector<double>>(
      "accelerometer_misalignment",
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0}
    );
    std::vector<double> accel_sensitivity = this->declare_parameter<std::vector<double>>(
      "accelerometer_sensitivity",
      {1.0, 1.0, 1.0}
    );
    std::vector<double> accel_offset = this->declare_parameter<std::vector<double>>(
      "accelerometer_offset",
      {0.0, 0.0, 0.0}
    );

    std::vector<double> soft_iron_matrix = this->declare_parameter<std::vector<double>>(
      "soft_iron_matrix",
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0}
    );
    std::vector<double> hard_iron_offset = this->declare_parameter<std::vector<double>>(
      "hard_iron_offset",
      {0.0, 0.0, 0.0}
    );

    // 3. Store them internally but DON'T apply them yet
    // We'll apply them only upon calibration service call
    calibration_data_.gyro_misalignment = gyro_misalignment;
    calibration_data_.gyro_sensitivity  = gyro_sensitivity;
    calibration_data_.gyro_offset       = gyro_offset;
    calibration_data_.accel_misalignment = accel_misalignment;
    calibration_data_.accel_sensitivity  = accel_sensitivity;
    calibration_data_.accel_offset       = accel_offset;
    calibration_data_.soft_iron_matrix   = soft_iron_matrix;
    calibration_data_.hard_iron_offset   = hard_iron_offset;

    // Identity calibration by default (until user triggers the service)
    setIdentityCalibration();

    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                                .best_effort();  // Use Best Effort instead of Reliable
    // 4. Create subscriptions
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_raw_topic_, qos_profile,
      std::bind(&ImuFusionNode::imuCallback, this, std::placeholders::_1)
    );

    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic_, qos_profile,
      std::bind(&ImuFusionNode::magCallback, this, std::placeholders::_1)
    );

    // Fused IMU publisher
    if (publish_fused_imu_) {
      fused_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(fused_imu_topic_, 10);
    }

    // TF broadcaster
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Calibration service
    calibration_service_ = this->create_service<imu_fusion_ros_interface::srv::CalibrationTrigger>(
      "trigger_calibration",
      std::bind(
        &ImuFusionNode::calibrationCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    // 5. Initialize AHRS
    FusionOffsetInitialise(&offset_, DEFAULT_SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs_);

    // 6. Configure AHRS settings from parameters
    FusionAhrsSettings settings;
    settings.convention = FusionConventionNwu;  // NWU or your desired convention
    settings.gain = (float)gain_;
    settings.gyroscopeRange = (float)gyro_range_;
    settings.accelerationRejection = (float)acceleration_rejection_;
    settings.magneticRejection = (float)magnetic_rejection_;
    settings.recoveryTriggerPeriod = recovery_trigger_period_;
    FusionAhrsSetSettings(&ahrs_, &settings);

    RCLCPP_INFO(this->get_logger(), "IMU Fusion Node initialized");
  }

private:

  // A struct to hold our calibration data from parameters
  struct CalibrationData {
    std::vector<double> gyro_misalignment;
    std::vector<double> gyro_sensitivity;
    std::vector<double> gyro_offset;

    std::vector<double> accel_misalignment;
    std::vector<double> accel_sensitivity;
    std::vector<double> accel_offset;

    std::vector<double> soft_iron_matrix;
    std::vector<double> hard_iron_offset;
  } calibration_data_;

  // For storing the "active" FusionMatrix / FusionVector objects once calibration is applied
  FusionMatrix gyroscope_misalignment_;
  FusionVector gyroscope_sensitivity_;
  FusionVector gyroscope_offset_;

  FusionMatrix accelerometer_misalignment_;
  FusionVector accelerometer_sensitivity_;
  FusionVector accelerometer_offset_;

  FusionMatrix soft_iron_matrix_;
  FusionVector hard_iron_offset_;

  // Flag to indicate if we've applied actual calibrations or not
  bool calibration_applied_ = false;

  // -----------------------
  // ROS Parameters
  std::string imu_raw_topic_;
  std::string mag_topic_;
  std::string fused_imu_topic_;

  bool publish_fused_imu_;
  bool publish_tf_;
  std::string map_frame_id_;
  std::string target_frame_id_;
  bool use_gravity_compensated_accel_;

  // AHRS settings
  double gain_;
  double gyro_range_;
  double acceleration_rejection_;
  double magnetic_rejection_;
  int recovery_trigger_period_;

  // -----------------------
  // ROS Entities
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fused_imu_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<imu_fusion_ros_interface::srv::CalibrationTrigger>::SharedPtr calibration_service_;

  // -----------------------
  // Fusion objects
  FusionOffset offset_;
  FusionAhrs ahrs_;

  // Buffers for sensor data
  FusionVector gyroscope_     = {0.0f, 0.0f, 0.0f};
  FusionVector accelerometer_ = {0.0f, 0.0f, 0.0f};
  FusionVector magnetometer_  = {0.0f, 0.0f, 0.0f};

  rclcpp::Time previous_time_;
  bool first_imu_msg_ = true;

  // ---------------------------------------------------------
  // Utility: set identity calibration (applied at startup)
  void setIdentityCalibration()
  {
    // Just set everything to identity or zero
    gyroscope_misalignment_ = (FusionMatrix){1.0f, 0.0f, 0.0f,
                                             0.0f, 1.0f, 0.0f,
                                             0.0f, 0.0f, 1.0f};
    gyroscope_sensitivity_  = (FusionVector){1.0f, 1.0f, 1.0f};
    gyroscope_offset_       = (FusionVector){0.0f, 0.0f, 0.0f};

    accelerometer_misalignment_ = (FusionMatrix){1.0f, 0.0f, 0.0f,
                                                 0.0f, 1.0f, 0.0f,
                                                 0.0f, 0.0f, 1.0f};
    accelerometer_sensitivity_  = (FusionVector){1.0f, 1.0f, 1.0f};
    accelerometer_offset_       = (FusionVector){0.0f, 0.0f, 0.0f};

    soft_iron_matrix_ = (FusionMatrix){1.0f, 0.0f, 0.0f,
                                       0.0f, 1.0f, 0.0f,
                                       0.0f, 0.0f, 1.0f};
    hard_iron_offset_ = (FusionVector){0.0f, 0.0f, 0.0f};

    calibration_applied_ = false;
    RCLCPP_INFO(this->get_logger(), "Using Identity calibration (startup).");
  }

  // ---------------------------------------------------------
  // Apply calibration from the stored parameters
  void applyCalibration()
  {
    // Convert from std::vector<double> to FusionMatrix, FusionVector
    if (calibration_data_.gyro_misalignment.size() == 9) {
      // Fill gyroscope misalignment
      gyroscope_misalignment_ = (FusionMatrix){
        (float)calibration_data_.gyro_misalignment[0],
        (float)calibration_data_.gyro_misalignment[1],
        (float)calibration_data_.gyro_misalignment[2],
        (float)calibration_data_.gyro_misalignment[3],
        (float)calibration_data_.gyro_misalignment[4],
        (float)calibration_data_.gyro_misalignment[5],
        (float)calibration_data_.gyro_misalignment[6],
        (float)calibration_data_.gyro_misalignment[7],
        (float)calibration_data_.gyro_misalignment[8]
      };
    }

    if (calibration_data_.gyro_sensitivity.size() == 3) {
      gyroscope_sensitivity_ = (FusionVector){
        (float)calibration_data_.gyro_sensitivity[0],
        (float)calibration_data_.gyro_sensitivity[1],
        (float)calibration_data_.gyro_sensitivity[2],
      };
    }

    if (calibration_data_.gyro_offset.size() == 3) {
      gyroscope_offset_ = (FusionVector){
        (float)calibration_data_.gyro_offset[0],
        (float)calibration_data_.gyro_offset[1],
        (float)calibration_data_.gyro_offset[2],
      };
    }

    // Accelerometer
    if (calibration_data_.accel_misalignment.size() == 9) {
      accelerometer_misalignment_ = (FusionMatrix){
        (float)calibration_data_.accel_misalignment[0],
        (float)calibration_data_.accel_misalignment[1],
        (float)calibration_data_.accel_misalignment[2],
        (float)calibration_data_.accel_misalignment[3],
        (float)calibration_data_.accel_misalignment[4],
        (float)calibration_data_.accel_misalignment[5],
        (float)calibration_data_.accel_misalignment[6],
        (float)calibration_data_.accel_misalignment[7],
        (float)calibration_data_.accel_misalignment[8]
      };
    }

    if (calibration_data_.accel_sensitivity.size() == 3) {
      accelerometer_sensitivity_ = (FusionVector){
        (float)calibration_data_.accel_sensitivity[0],
        (float)calibration_data_.accel_sensitivity[1],
        (float)calibration_data_.accel_sensitivity[2]
      };
    }

    if (calibration_data_.accel_offset.size() == 3) {
      accelerometer_offset_ = (FusionVector){
        (float)calibration_data_.accel_offset[0],
        (float)calibration_data_.accel_offset[1],
        (float)calibration_data_.accel_offset[2]
      };
    }

    // Magnetometer
    if (calibration_data_.soft_iron_matrix.size() == 9) {
      soft_iron_matrix_ = (FusionMatrix){
        (float)calibration_data_.soft_iron_matrix[0],
        (float)calibration_data_.soft_iron_matrix[1],
        (float)calibration_data_.soft_iron_matrix[2],
        (float)calibration_data_.soft_iron_matrix[3],
        (float)calibration_data_.soft_iron_matrix[4],
        (float)calibration_data_.soft_iron_matrix[5],
        (float)calibration_data_.soft_iron_matrix[6],
        (float)calibration_data_.soft_iron_matrix[7],
        (float)calibration_data_.soft_iron_matrix[8]
      };
    }

    if (calibration_data_.hard_iron_offset.size() == 3) {
      hard_iron_offset_ = (FusionVector){
        (float)calibration_data_.hard_iron_offset[0],
        (float)calibration_data_.hard_iron_offset[1],
        (float)calibration_data_.hard_iron_offset[2]
      };
    }

    calibration_applied_ = true;
    RCLCPP_INFO(this->get_logger(), "Calibration data has been applied!");
  }

  // Utility to rotate a vector from Earth frame into Sensor frame
  // using the conjugate of the sensor->Earth quaternion.
  // (Because FusionAhrsGetQuaternion returns orientation of sensor relative to Earth.)
  FusionVector rotateEarthToSensor(const FusionVector& earth_vec, const FusionQuaternion& sensor_to_earth) {
    // 1. Conjugate sensor->earth
    FusionQuaternion conj;
    conj.element.w = sensor_to_earth.element.w;
    conj.element.x = -sensor_to_earth.element.x;
    conj.element.y = -sensor_to_earth.element.y;
    conj.element.z = -sensor_to_earth.element.z;
  
    // 2. Convert earth_vec to a quaternion with w=0
    FusionQuaternion vec_quat;
    vec_quat.element.w = 0.0f;
    vec_quat.element.x = earth_vec.axis.x;
    vec_quat.element.y = earth_vec.axis.y;
    vec_quat.element.z = earth_vec.axis.z;
  
    // 3. Multiply conj * vec_quat
    FusionQuaternion tmp = FusionQuaternionMultiply(conj, vec_quat);
  
    // 4. Multiply the result by the original sensor->earth quaternion
    FusionQuaternion out = FusionQuaternionMultiply(tmp, sensor_to_earth);
  
    // 5. Extract x,y,z into a FusionVector
    FusionVector result;
    result.axis.x = out.element.x;
    result.axis.y = out.element.y;
    result.axis.z = out.element.z;
  
    return result;
  }

  // -----------------------------------------------------------
  // Callback for raw IMU data (accelerometer & gyroscope)
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Convert sensor_msgs/Imu into FusionVector
    gyroscope_.axis.x = msg->angular_velocity.x * 180.0f / 3.14159f;  // rad/s -> deg/s
    gyroscope_.axis.y = msg->angular_velocity.y * 180.0f / 3.14159f;
    gyroscope_.axis.z = msg->angular_velocity.z * 180.0f / 3.14159f;

    accelerometer_.axis.x = msg->linear_acceleration.x / 9.81f; // m/s^2 -> g
    accelerometer_.axis.y = msg->linear_acceleration.y / 9.81f;
    accelerometer_.axis.z = msg->linear_acceleration.z / 9.81f;

    // Only apply calibration if service triggered
    if (calibration_applied_) {
      // Apply inertial calibration
      gyroscope_ = FusionCalibrationInertial(
        gyroscope_,
        gyroscope_misalignment_,
        gyroscope_sensitivity_,
        gyroscope_offset_
      );

      accelerometer_ = FusionCalibrationInertial(
        accelerometer_,
        accelerometer_misalignment_,
        accelerometer_sensitivity_,
        accelerometer_offset_
      );
    }

    // Offset correction
    gyroscope_ = FusionOffsetUpdate(&offset_, gyroscope_);

    // Delta time
    float dt = 1.0f / DEFAULT_SAMPLE_RATE;
    if (!first_imu_msg_) {
      double current_time  = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
      double previous_time = previous_time_.seconds();
      dt = static_cast<float>(current_time - previous_time);
    } else {
      first_imu_msg_ = false;
    }
    previous_time_ = msg->header.stamp;

    // Update AHRS with or without magnetometer
    FusionAhrsUpdate(&ahrs_, gyroscope_, accelerometer_, magnetometer_, dt);

    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs_);

    // Publish fused IMU
    if (publish_fused_imu_ && fused_imu_pub_) {
      sensor_msgs::msg::Imu fused_msg;
      fused_msg.header.stamp = this->get_clock()->now();
      fused_msg.header.frame_id = target_frame_id_;

      fused_msg.orientation.x = quat.element.x;
      fused_msg.orientation.y = quat.element.y;
      fused_msg.orientation.z = quat.element.z;
      fused_msg.orientation.w = quat.element.w;


      // Fill angular_velocity in [rad/s] using our offset-corrected gyroscope_ (deg/s -> rad/s)
      fused_msg.angular_velocity.x = gyroscope_.axis.x * static_cast<float>(M_PI) / 180.0f;
      fused_msg.angular_velocity.y = gyroscope_.axis.y * static_cast<float>(M_PI) / 180.0f;
      fused_msg.angular_velocity.z = gyroscope_.axis.z * static_cast<float>(M_PI) / 180.0f;

      // Fill linear_acceleration in [m/s^2].
      // We start with the calibrated accelerometer in [g], then multiply by 9.81.
      float ax = accelerometer_.axis.x * 9.80665f;
      float ay = accelerometer_.axis.y * 9.80665f;
      float az = accelerometer_.axis.z * 9.80665f;

      if (use_gravity_compensated_accel_) {
        // Subtract gravity in the sensor frame
        // Earth frame gravity (NWU convention) is [0, 0, 9.81]
        FusionVector gravity_earth = {0.0f, 0.0f, 9.80665f};
        FusionVector gravity_sensor = rotateEarthToSensor(gravity_earth, quat);
        ax -= gravity_sensor.axis.x;
        ay -= gravity_sensor.axis.y;
        az -= gravity_sensor.axis.z;
      }

      fused_msg.linear_acceleration.x = ax;
      fused_msg.linear_acceleration.y = ay;
      fused_msg.linear_acceleration.z = az;

      fused_imu_pub_->publish(fused_msg);
    }

    // Publish TF
    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->get_clock()->now();
      transform.header.frame_id = map_frame_id_;
      transform.child_frame_id = target_frame_id_;

      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;

      transform.transform.rotation.x = quat.element.x;
      transform.transform.rotation.y = quat.element.y;
      transform.transform.rotation.z = quat.element.z;
      transform.transform.rotation.w = quat.element.w;

      tf_broadcaster_->sendTransform(transform);
    }
  }

  // -----------------------------------------------------------
  // Callback for magnetometer data
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    magnetometer_.axis.x = msg->magnetic_field.x;
    magnetometer_.axis.y = msg->magnetic_field.y;
    magnetometer_.axis.z = msg->magnetic_field.z;

    // Only apply calibration if service triggered
    if (calibration_applied_) {
      magnetometer_ = FusionCalibrationMagnetic(
        magnetometer_,
        soft_iron_matrix_,
        hard_iron_offset_
      );
    }
  }

  // -----------------------------------------------------------
  // Service callback to trigger calibration
  void calibrationCallback(
    const std::shared_ptr<imu_fusion_ros_interface::srv::CalibrationTrigger::Request> request,
    std::shared_ptr<imu_fusion_ros_interface::srv::CalibrationTrigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Calibration triggered: %s", request->calibration_type.c_str());

    // For simplicity, we always apply the same calibration data from YAML,
    // ignoring the type for now. You could differentiate by 'magnetometer', etc.
    // or store multiple sets of data.
    applyCalibration();

    response->success = true;
    response->message = "Calibration applied from parameters!";
  }
};

// -----------------------------------------------------------
// main
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFusionNode>());
  rclcpp::shutdown();
  return 0;
}
