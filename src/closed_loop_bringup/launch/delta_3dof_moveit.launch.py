"""Plan motions for the closed-loop delta_3dof in MoveIt/RViz and execute them in Gazebo.

Same pipeline as fivebar_moveit.launch.py, but the moving platform (platform_x/y/z -> tool0) is
already in the base tree and IS the MoveIt planning chain, so there is no phantom 'planning' render.
Renders of the single 3dof_delta xacro:
  * sim + ros2_control     -> robot_state_publisher / Gazebo / gz_ros2_control
  * sim:=false             -> move_group's planning model (platform = base_link -> tool0)
  * sim:=false constraints -> the transformer's robot_model (closed-loop solver)
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

ROBOT = "delta_3dof"
XACRO = "3dof_delta.urdf.xacro"
CONTROLLERS = "delta_3dof_controllers.yaml"


def generate_launch_description():
    desc_pkg = get_package_share_directory("closed_loop_description")
    robot_xacro = os.path.join(desc_pkg, "urdf", XACRO)
    controllers_yaml = os.path.join(desc_pkg, "config", CONTROLLERS)

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="delta_world")
    gui_arg = DeclareLaunchArgument(name="gui", default_value="true",
                                    description="Run the Gazebo GUI (false = server only / headless)")
    rviz_arg = DeclareLaunchArgument(name="rviz", default_value="true",
                                     description="Launch RViz MotionPlanning")
    world_path = PathJoinSubstitution(
        [desc_pkg, "worlds",
         PythonExpression(["'", LaunchConfiguration("world_name"), "' + '.sdf'"])]
    )

    resource_path = str(Path(desc_pkg).parent.resolve()) + pathsep + os.path.join(desc_pkg, "meshes")
    gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path)

    # --- renders of the one xacro ---------------------------------------------------------
    sim_urdf = xacro.process_file(
        robot_xacro, mappings={"sim": "true", "ros2_control": "true",
                               "controllers_file": controllers_yaml}).toxml()
    constraint_urdf = xacro.process_file(
        robot_xacro, mappings={"sim": "false", "constraints": "true"}).toxml()

    # --- Gazebo + the spawned, gz_ros2_control-enabled robot -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": sim_urdf, "use_sim_time": True}],
    )
    gui_config = os.path.join(desc_pkg, "config", "delta_3dof_gui.config")  # isometric initial camera
    gz_flags = PythonExpression(["'-s -r' if '", LaunchConfiguration("gui"), "' == 'false' else '-r'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={"gz_args": PythonExpression(
            ["' ", world_path, " -v 4 --gui-config ", gui_config, " ' + '", gz_flags, "'"])}.items(),
    )
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", ROBOT],
    )
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # --- ros2_control spawners -----------------------------------------------------------
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

    # --- MoveIt move_group + RViz (planning model = base tree, sim:=false) ----------------
    moveit_config = (
        MoveItConfigsBuilder(ROBOT, package_name="closed_loop_moveit_config")
        .robot_description(file_path=robot_xacro, mappings={"sim": "false"})
        .robot_description_semantic(file_path=f"config/{ROBOT}.srdf")
        .joint_limits(file_path=f"config/{ROBOT}_joint_limits.yaml")
        .trajectory_execution(file_path=f"config/{ROBOT}_moveit_controllers.yaml")
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
        arguments=["-d", os.path.join(get_package_share_directory("closed_loop_moveit_config"),
                                      "config", "delta_3dof.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

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
