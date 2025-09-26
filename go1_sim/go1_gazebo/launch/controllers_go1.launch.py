import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    robot_description_config = os.path.join(
        get_package_share_directory("go1_description"),
        "urdf",
        "go1.urdf"  # make sure this is the correct URDF
    )
    robot_description = {'robot_description': open(robot_description_config).read()}

    controller_yaml = os.path.join(
        get_package_share_directory("go1_control"),
        "config",
        "go1_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_yaml, {"use_sim_time": True}],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Other spawners...
    FR_hip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FR_hip_controller", "-c", "/controller_manager"],
    )

    # Add remaining controllers like you did before...

    return LaunchDescription([
        ros2_control_node,  # <<<< ✅ YOU WERE MISSING THIS
        joint_state_broadcaster_spawner,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[FR_hip_controller_spawner],
            )
        ),

        # All other RegisterEventHandler blocks...
    ])
