import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo','/root/.gazebo/worlds/ttest'],
        output="screen"
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # arguments=[-d,""]
    )
    teleop_node =Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_node",
        remappings=[
            ("cmd_vel", "demo/cmd_demo")]
    )

    pcl_node = Node(
        package="pcl_process",
        executable="pc_boundary",
        name="pc_boundary",
        output="screen",
        parameters=[
                {'KS': 50},
                {'RS':0.1},
                {'AT':1.2}
                ]
    )
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/demo/cmd_demo


    ld.add_action(start_gazebo_cmd)
    # ld.add_action(start_rviz_cmd)
    # ld.add_action(teleop_node)
    # ld.add_action(pcl_node)

    return ld