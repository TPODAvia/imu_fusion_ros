import os
from dotenv import load_dotenv
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def parse_bool_env(var_name, default="False"):
    val = os.getenv(var_name, default)
    return val.lower() in ("true", "1", "yes")

def parse_float_env(var_name, default="0.0"):
    val = os.getenv(var_name, default)
    return float(val)

def parse_str_env(var_name, default=""):
    return os.getenv(var_name, default)

def generate_launch_description():

    # Relative path to the .env file
    env_file_path = os.path.expanduser("~/gr_platform2/.env")
    if os.path.exists(env_file_path):
        load_dotenv(env_file_path)
    else:
        print(f"Warning: .env file not found at {env_file_path}")

    # Retrieve environment variables or defaults
    publish_fused_imu = parse_bool_env("PUBLISH_FUSED_IMU", "True")
    publish_tf        = parse_bool_env("PUBLISH_TF", "True")
    map_frame_id      = parse_str_env("MAP_FRAME_ID", "map")
    target_frame_id   = parse_str_env("TARGET_FRAME_ID", "base_link")
    imu_raw_topic     = parse_str_env("IMU_RAW_TOPIC", "imu/data_raw")
    mag_topic         = parse_str_env("MAG_TOPIC", "imu/mag")
    fused_imu_topic   = parse_str_env("FUSED_IMU_TOPIC", "imu/data")

    # AHRS parameters from environment (or fallback)
    gain                   = parse_float_env("AHRS_GAIN", "0.5")
    gyro_range             = parse_float_env("AHRS_GYRO_RANGE", "2000.0")
    acceleration_rejection = parse_float_env("AHRS_ACC_REJECTION", "10.0")
    magnetic_rejection     = parse_float_env("AHRS_MAG_REJECTION", "10.0")
    recovery_trigger       = int(parse_float_env("AHRS_RECOVERY_TRIGGER_PERIOD", "500"))

    # Where the calibration YAML is stored:
    calibration_file = os.path.join(
        get_package_share_directory('imu_fusion_ros'),
        'config',
        'calibration.yaml'
    )

    return LaunchDescription([
        # Optional: declare the path to the YAML file, so it is used by the node
        DeclareLaunchArgument(
            'calibration_file',
            default_value=calibration_file,
            description='Path to calibration YAML file.'
        ),

        Node(
            package='imu_fusion_ros',
            executable='imu_fusion_node',
            name='imu_fusion_node',
            output='screen',
            parameters=[
                # Dynamic parameters from .env
                {'publish_fused_imu': publish_fused_imu},
                {'publish_tf': publish_tf},
                {'map_frame_id': map_frame_id},
                {'target_frame_id': target_frame_id},
                {'imu_raw_topic': imu_raw_topic},
                {'mag_topic': mag_topic},
                {'fused_imu_topic': fused_imu_topic},

                # AHRS settings
                {'gain': gain},
                {'gyro_range': gyro_range},
                {'acceleration_rejection': acceleration_rejection},
                {'magnetic_rejection': magnetic_rejection},
                {'recovery_trigger_period': recovery_trigger},

                # Load calibration arrays from YAML
                LaunchConfiguration('calibration_file')
            ]
        ),
    ])

def get_package_share_directory(pkg_name):
    # If you have 'ament_index_python' or other means, import from there.
    # Or just manually define the path
    # For example:
    return os.path.join(os.getenv('COLCON_PREFIX_PATH', '/tmp'), pkg_name)
