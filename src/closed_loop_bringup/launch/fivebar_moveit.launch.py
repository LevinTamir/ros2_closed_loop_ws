"""Plan motions for the closed-loop fivebar_linkage in MoveIt/RViz and execute them in Gazebo.

Pipeline (see closed_loop_kinematics/BUILD.md and the description xacros):

  MoveIt move_group  --(plan phantom tool trajectory)-->  joint_state_transformer
        |                                                          |  IK (robot_model + LM)
        |                                                          v
        |                              actuated_joint_trajectory_controller (gz_ros2_control)
        |                                                          |  drives the 2 motors in Gazebo
        v                                                          v
   RViz MotionPlanning                              DetachableJoint closes the 5-bar loop
        ^                                                          |
        |  full /joint_states (legs via FK + tool pose)           |  measured active joints
        +------------------ joint_state_transformer <--- joint_state_broadcaster

Three renders of the single fivebar xacro are used:
  * sim  + ros2_control  -> robot_state_publisher / Gazebo / gz_ros2_control  (global /robot_description)
  * planning             -> move_group's robot_description (the phantom planning model)
  * planning + constraints -> the transformer's robot_model (closed-loop solver)
"""
import os
from pathlib import Path
from os import pathsep

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch_ros.actions import Node
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_pkg = get_package_share_directory("closed_loop_description")
    fivebar_xacro = os.path.join(desc_pkg, "urdf", "fivebar_linkage.urdf.xacro")
    controllers_yaml = os.path.join(desc_pkg, "config", "fivebar_controllers.yaml")

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="fivebar_world")
    gui_arg = DeclareLaunchArgument(name="gui", default_value="true",
                                    description="Run the Gazebo GUI (false = server only / headless)")
    rviz_arg = DeclareLaunchArgument(name="rviz", default_value="true",
                                     description="Launch RViz MotionPlanning")
    world_path = PathJoinSubstitution(
        [desc_pkg, "worlds",
         PythonExpression(["'", LaunchConfiguration("world_name"), "' + '.sdf'"])]
    )

    # Gazebo needs to find meshes (package:// and the workspace root)
    resource_path = str(Path(desc_pkg).parent.resolve()) + pathsep + os.path.join(desc_pkg, "meshes")
    gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path)

    # --- three renders of the one xacro ---------------------------------------------------
    sim_urdf = xacro.process_file(
        fivebar_xacro, mappings={"sim": "true", "ros2_control": "true",
                                 "controllers_file": controllers_yaml}).toxml()
    planning_urdf = xacro.process_file(
        fivebar_xacro, mappings={"sim": "false", "planning": "true"}).toxml()
    constraint_urdf = xacro.process_file(
        fivebar_xacro, mappings={"sim": "false", "planning": "true", "constraints": "true"}).toxml()

    # --- Gazebo + the spawned, gz_ros2_control-enabled robot -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": sim_urdf, "use_sim_time": True}],
    )
    gz_flags = PythonExpression(["'-s -r' if '", LaunchConfiguration("gui"), "' == 'false' else '-r'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={"gz_args": PythonExpression(["' ", world_path, " -v 4 ' + '", gz_flags, "'"])}.items(),
    )
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "fivebar_linkage"],
    )
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # --- ros2_control spawners (gz_ros2_control hosts the controller_manager) -------------
    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    jtc_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["actuated_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # --- joint_state_transformer + its (constraint-render) model -------------------------
    transformer_description = Node(
        package="closed_loop_bringup", executable="description_publisher", name="transformer_description",
        parameters=[{"robot_description": constraint_urdf, "topic": "robot_description"}],
        remappings=[("robot_description", "/transformer/robot_description")],
    )
    transformer = Node(
        package="closed_loop_bringup", executable="transformer_with_preview", name="joint_state_transformer",
        output="screen",
        parameters=[{
            "tolerance": 1e-4,
            "max_iterations": 200,
            "max_step_line_search": 20,
            "input_action_name": "~/follow_joint_trajectory",
            "output_action_name": "/actuated_joint_trajectory_controller/follow_joint_trajectory",
            "output_joint_states_topic": "/joint_states",
            "use_sim_time": True,
        }],
        remappings=[("/robot_description", "/transformer/robot_description")],
    )

    # --- MoveIt move_group + RViz (planning render) --------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("fivebar_linkage", package_name="fivebar_moveit_config")
        .robot_description(file_path=fivebar_xacro,
                           mappings={"sim": "false", "planning": "true"})
        .robot_description_semantic(file_path="config/fivebar_linkage.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(get_package_share_directory("fivebar_moveit_config"),
                                      "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # spawn controllers only once the robot is in Gazebo; start move_group/rviz after jsb
    # Controllers must wait for gz_ros2_control (loaded when the robot spawns). move_group and
    # RViz do NOT depend on the controllers to start (only to execute later), so launch them
    # immediately, in parallel with Gazebo, so the RViz window opens right away.
    spawn_controllers_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner]))
    jtc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[jtc_spawner]))

    return LaunchDescription([
        world_name_arg, gui_arg, rviz_arg, gz_resource,
        robot_state_publisher, gazebo, spawn, clock_bridge,
        transformer_description, transformer,
        move_group, rviz,
        spawn_controllers_after_robot, jtc_after_jsb,
    ])
