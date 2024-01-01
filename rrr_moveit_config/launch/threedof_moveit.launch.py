#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import rospkg
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import yaml
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():
    # Declare a launch argument for the model URDF file
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("rrr_description"), "urdf", "robot.urdf.xacro"),
        description="Absolute path to the robot URDF file")
    # Get the robot description from the 'model' argument
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("rrr_description"), "share"))
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
    print(robot_description.value)

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"))
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "manipulator", "-topic", "robot_description"]
    )

    # Define a node to publish the robot state
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}])

    # ***** CONTROLLERS ***** #
    # Joint state broadcaster:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Joint trajectory controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["finger1_controller", "-c", "/controller_manager"],
    )

    right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["finger2_controller", "-c", "/controller_manager"],
    )

    # *********************** MoveIt!2 *********************** #   

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    robot_description_semantic_config = load_file("rrr_moveit_config", "config/manipulator.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config }


    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("rrr_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("rrr_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    #MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("rrr_moveit_config", "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    pkg_path = os.path.join(get_package_share_directory('rrr_description'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    robot_description_config = doc.toxml()
    robot_description_str = {'robot_description': robot_description_config}

     # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_str,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'publish_robot_description_semantic': True},
            {"use_sim_time": True}

        ],
    )
    print("9")

    # RVIZ:
    rviz_base = os.path.join(get_package_share_directory("rrr_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description_str,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {"use_sim_time": True}, 
        ],
    )

    return LaunchDescription([
        model_arg,
        env_var,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_robot,
                on_exit = [
                    joint_state_broadcaster_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_state_broadcaster_spawner,
                on_exit = [
                    joint_trajectory_controller_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_trajectory_controller_spawner,
                on_exit = [
                        left_controller_spawner,
                    ]
                )
            ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = left_controller_spawner,
                on_exit = [
                        right_controller_spawner,
                    ]
                )
            ),

        RegisterEventHandler(
            OnProcessExit(
                target_action = right_controller_spawner,
                on_exit = [

                        # MoveIt!2:
                        TimerAction(
                            period=5.0,
                            actions=[
                                rviz_node_full,
                                run_move_group_node
                            ]
                        ),

                    ]
                )
            )
        
    ])