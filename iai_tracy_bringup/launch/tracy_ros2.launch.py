from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch args for IPs
    left_ip = LaunchConfiguration('left_robot_ip')
    right_ip = LaunchConfiguration('right_robot_ip')

    tracy_xacro_file = os.path.join(get_package_share_directory('iai_tracy_description'), 'urdf',
                                     'tracy.urdf.xacro')

    left_kinematics = os.path.join(
        get_package_share_directory('iai_tracy_ur'), 'config', 'left_arm_calibration.yaml')

    right_kinematics = os.path.join(
        get_package_share_directory('iai_tracy_ur'), 'config', 'right_arm_calibration.yaml')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', tracy_xacro_file,
        ' kinematics_config_left:=', left_kinematics,
        ' kinematics_config_right:=', right_kinematics,
    ])

    # Get the orbbec_camera package directory
    orbbec_share_dir = get_package_share_directory('orbbec_camera')
    orbbec_launch_dir = os.path.join(orbbec_share_dir, 'launch')


    return LaunchDescription([
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.102.154'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.102.153'),

        # LEFT ARM
        GroupAction([
            PushRosNamespace('left_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('iai_tracy_bringup'),
                        'launch',
                        'iai_ur_control.launch.py'
                    )
                ]),
                launch_arguments={
                    'robot_ip': left_ip,
                    'use_fake_hardware': 'false',
                    'ur_type': 'ur10e',
                    'tf_prefix': 'left_',
                    'initial_joint_controller': 'forward_velocity_controller',
                    'launch_rviz': 'false',
                    'reverse_port': '50011',
                    'script_sender_port': '50012',
                    'trajectory_port': '50013',
                    'script_command_port': '50014',
                    'kinematics_params_file': left_kinematics,
                    'controllers_file': os.path.join(
                        get_package_share_directory('iai_tracy_ur'),
                        'config',
                        'ur10e_controllers_left.yaml'
                    ),
                }.items()
            ),
        ]),

        # RIGHT ARM
        GroupAction([
            PushRosNamespace('right_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('iai_tracy_bringup'),
                        'launch',
                        'iai_ur_control.launch.py'
                    )
                ]),
                launch_arguments={
                    'robot_ip': right_ip,
                    'use_fake_hardware': 'false',
                    'ur_type': 'ur10e',
                    'tf_prefix': 'right_',
                    'initial_joint_controller': 'forward_velocity_controller',
                    'launch_rviz': 'false',
                    'reverse_port': '50001',
                    'script_sender_port': '50002',
                    'trajectory_port': '5003',
                    'script_command_port': '50005',
                    'kinematics_params_file': right_kinematics,
                    'controllers_file': os.path.join(
                        get_package_share_directory('iai_tracy_ur'),
                        'config',
                        'ur10e_controllers_right.yaml'
                    ),
                }.items()
            ),
        ]),
        # DUAL ROBOTIQ GRIPPERS SETUP
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('robotiq_description'),
                    'launch',
                    'robotiq.launch.py'
                )
            ]),
            launch_arguments={
                'is_dual': 'true',
                'gripper_type': '140',
                'use_fake_hardware': 'false',
                'com_port_left': '/dev/ttyUSB1',
                'com_port_right': '/dev/ttyUSB0',
            }.items(),
        ),

        # Camera
        #GroupAction([
        #    PushRosNamespace('tracy_camera'),
        #    IncludeLaunchDescription(
        #        PythonLaunchDescriptionSource([
        #            os.path.join(
        #                get_package_share_directory('realsense2_camera'),
        #                'launch',
        #                'rs_launch.py'
        #            )
        #        ]),
        #        launch_arguments={
        #             'depth_module.depth_profile': '1280x720x30',  # <<< CHANGE or REMOVE
        #             'rgb_camera.color_profile': '1280x720x30',  # <<< CHANGE or REMOVE
        #        #     # Add more launch arguments as needed
        #        }.items(),
        #    )
        #]),
        # Orbbec Camera
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(orbbec_launch_dir, 'femto_mega.launch.py')
           ),
           launch_arguments={
               'color_width': '1920',
               'color_height': '1080',
               'depth_registration': 'True',
               'enable_colored_point_cloud': 'True',
               'enable_noise_removal_filter': 'True',
               'noise_removal_filter_min_diff': '3',
           }.items()
        ),

        # JOINT STATE PUBLISHER (merged)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': [
                    '/left_arm/joint_states',
                    '/right_arm/joint_states',
                    '/left_gripper/joint_states',
                    '/right_gripper/joint_states'
                ],
                'rate': 100.0,
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            remappings=[('/joint_states', '/asdf')],# remapping to asdf because the RSP should only publish static transforms
            parameters=[{'robot_description': robot_description}]
        )
    ])
