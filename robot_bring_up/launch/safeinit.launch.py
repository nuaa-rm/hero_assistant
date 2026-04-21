from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import TimerAction

def generate_launch_description():
    if_rviz = True
    if_sim = False
    if_map = False

    point_lio_path = get_package_share_directory("point_lio")
    robot_bringup_path = get_package_share_directory("robot_bring_up")
    obstacle_segmentation_path = get_package_share_directory("obstacle_segmentation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    livox_driver_path = get_package_share_directory("livox_ros_driver2")

    param_if_map = LaunchConfiguration("if_map", default=if_map)
    declare_if_map = DeclareLaunchArgument(
        "if_map",
        default_value=param_if_map,
        description="Whether to run map",
    )
    param_yaml_path = LaunchConfiguration("params_file")
    declare_yaml_path = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(robot_bringup_path, "config", "safeinitblue.yaml"),
        description="Full path to the configuration file to load",
    )
    param_launch_rviz = LaunchConfiguration("launch_rviz", default=if_rviz)
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value=param_launch_rviz,
        description="Whether to run rviz",
    )
    param_rviz_config_dir = LaunchConfiguration(
        "rviz_config_dir",
        default=os.path.join(robot_bringup_path,"config","betterRvizConfig.rviz"),
    )
    declare_rviz_config_dir = DeclareLaunchArgument(
        "rviz_config_dir",
        default_value=param_rviz_config_dir,
        description="Full path to the rviz config file to load",
    )
    param_launch_gazebo = LaunchConfiguration("launch_gazebo", default=if_sim)
    declare_launch_gazebo = DeclareLaunchArgument(
        "launch_gazebo",
        default_value=param_launch_gazebo,
        description="Whether to run gazebo",
    )

    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [livox_driver_path, "/launch_ROS2", "/msg_MID360_launch.py"]
        ),
    )

    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [point_lio_path, "/launch", "/pointlio.launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
        }.items(),
    )

    obstacle_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [obstacle_segmentation_path, "/launch", "/obstacle_segmentation.launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
        }.items(),
        condition=UnlessCondition(param_if_map),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_bringup_path, "/launch", "/bringup_launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
            "use_sim_time": param_launch_gazebo,
        }.items(),
        condition=UnlessCondition(param_if_map),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", param_rviz_config_dir],
        parameters=[{"use_sim_time": param_launch_gazebo}],
        output="screen",
        condition=IfCondition(param_launch_rviz),
    )

    origin2map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='RMmap_origin2map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0','1',  'origin', 'map']
    )

    livox_to_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox2body_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'livox_frame']
    )

    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    list = [
        livox_driver_launch,
        origin2map,
        livox_to_body,
        odom_to_base_link,
        declare_launch_gazebo,
        declare_yaml_path,
        declare_launch_rviz,
        declare_if_map,
        declare_rviz_config_dir,
        navigation_launch,
        rviz_node,
        TimerAction(
            period=5.0,
            actions=[
                point_lio_launch
            ],
        ),
        obstacle_segmentation_launch,
    ]

    return LaunchDescription(list)