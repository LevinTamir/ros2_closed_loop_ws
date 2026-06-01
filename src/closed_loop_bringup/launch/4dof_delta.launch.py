import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    closed_loop_description = get_package_share_directory("closed_loop_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(closed_loop_description, "urdf", "4dof_delta.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="delta_world")

    world_path = PathJoinSubstitution(
        [
            closed_loop_description,
            "worlds",
            PythonExpression(
                expression=["'", LaunchConfiguration("world_name"), "'", " + '.sdf'"]
            ),
        ]
    )

    model_path = str(Path(closed_loop_description).parent.resolve())
    model_path += pathsep + os.path.join(closed_loop_description, "meshes")

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    robot_description_content = xacro.process_file(
        os.path.join(closed_loop_description, "urdf", "4dof_delta.urdf.xacro")
    ).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments={
            "gz_args": PythonExpression(["' ", world_path, " -v 4 -r'"])
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "deltaarm_4dof",
        ],
    )

    # Gazebo publishes only the actuated joints (the URDF JointStatePublisher is filtered),
    # mimicking real encoders. Bridge that back to ROS and remap it to /joint_states.
    gz_joint_state_topic = "/world/delta_world/model/deltaarm_4dof/joint_state"

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/delta_4dof/Chain1_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/delta_4dof/Chain2_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/delta_4dof/Chain3_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/delta_4dof/Chain4_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            gz_joint_state_topic + "@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[(gz_joint_state_topic, "/joint_states")],
    )

    return LaunchDescription(
        [
            model_arg,
            world_name_arg,
            robot_state_publisher_node,
            gazebo_resource_path,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
        ]
    )
