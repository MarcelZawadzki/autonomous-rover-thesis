from launch_ros.actions     import Node
from launch                 import LaunchDescription
from launch.substitutions   import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros
import os
import math
import time


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_lidar').find('rover_lidar')
    default_model_path = os.path.join(pkg_share, 'description/rover_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    default_slam_config_path = os.path.join(pkg_share, 'config/slam.yaml')
    default_nav2_config_path = os.path.join(pkg_share, 'config/nav2_params.yaml')
    default_cartographer_folder = os.path.join(pkg_share, 'config')
    default_cartographer_file = 'cartographer3D.lua'
    no_gps_cartographer_file = 'cartographer3DnoGPS.lua'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rover_controller_node = Node(
        package='rover_lidar',
        executable='controller',
        name='controller',
        parameters=[{'gps': LaunchConfiguration('gps')}]
    )

    lidar_node = Node(
        package='rover_lidar',
        executable='scanner',
        name='scanner'
    )

    fake_odom_node = Node(
        package='rover_lidar',
        executable='fake_odom',
        name='fake_odom'
    )

    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        parameters=[{
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'use_mag': True,
            'gain_acc': 0.01,
            'gain_mag': 0.01,
            'publish_debug_topics' : False
        }]
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                   {'use_sim_time': False}]
    )

    cartographer_occupancy_grid_node = Node(
        arguments=['-configuration_directory', '0.05',
                   '-publish_period_sec', '1.0'],
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='log'
    )

    cartographer_node = Node(
        arguments=['-configuration_directory', default_cartographer_folder,
                   '-configuration_basename', default_cartographer_file],
        remappings=[('/points2', '/lidar_scan'),
                    ('/imu', '/imu/data'),
                    ('/fix', '/gps_rover'),
                    ('/odom', '/odometry/filtered')],
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        condition = IfCondition(LaunchConfiguration('gps'))
    )

    cartographer_node_no_gps = Node(
        arguments=['-configuration_directory', default_cartographer_folder,
                   '-configuration_basename', no_gps_cartographer_file],
        remappings=[('/points2', '/lidar_scan'),
                    ('/imu', '/imu/data'),
                    ('/fix', '/gps_rover'),
                    ('/odom', '/odometry/filtered')],
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        condition = UnlessCondition(LaunchConfiguration('gps'))
    )

    nav2_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                launch_ros.substitutions.FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items()
    )

    nav2_voxel_visualize = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_markers',
        name='nav2_costmap_2d_markers',
        remappings=[('voxel_grid', '/local_costmap/voxel_grid'),
                    ('visualization_marker', '/my_marker')],
    )

    delay_nav2_launch = TimerAction(
        period = 5.0,
        actions = [ nav2_launch ]
    )


    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui',
                                             default_value='True',
                                             description='Flag to enable GUI elements (False for headless)'),
        launch.actions.DeclareLaunchArgument(name='gps',
                                             default_value='True',
                                             description='Whether to use GPS (outdoor) or not (indoor)'),
        launch.actions.DeclareLaunchArgument(name='model',
                                             default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig',
                                             default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        # launch.actions.DeclareLaunchArgument(name='slam_params_file',
        #                                      default_value=default_slam_config_path,
        #                                      description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
        launch.actions.DeclareLaunchArgument(name='nav2_params_file',
                                             default_value=default_nav2_config_path,
                                             description='Full path to the ROS2 parameters file to use for nav2'),
        # launch.actions.DeclareLaunchArgument(name='cartographer_folder',
        #                                      default_value=default_cartographer_folder,
        #                                      description='Full path to the folder of cartographer config files'),
        # launch.actions.DeclareLaunchArgument(name='cartographer_file',
        #                                      default_value=(default_cartographer_file if IfCondition(LaunchConfiguration('gps'))._predicate_func()
        #                                                     else no_gps_cartographer_file),
        #                                      description='Filename of the main cartographer config file'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        rover_controller_node,
        lidar_node,
        imu_filter_node,
        robot_localization_node,
        fake_odom_node,

        ##################
        #cartographer slam
        ##################
        cartographer_occupancy_grid_node,
        cartographer_node,
        cartographer_node_no_gps,

        ##################
        #NAV 2
        ##################
        delay_nav2_launch,
        #nav2_voxel_visualize,
    ])
