from explorer_bringup.launch.controller_manager_spawner import (
    declare_node_gripper_controller_spawner,
    declare_node_qcontrol_controller_spawner,
)
from explorer_bringup.launch.hardware import (
    declare_hardware_node_group,
)
from explorer_bringup.launch.hardware_parameters import declare_hardware_argument_list
from explorer_bringup.launch.simulation import (
    declare_simulation_node_group,
)
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    robot_controller_config = "explorer_params"
    robot_controller_config_path = "sandbox_controller"

    # --------------------------------------------------------------------------
    # Configuration & Arguments
    # --------------------------------------------------------------------------
    declared_arguments = [
        ## Override subsequent arg declaration by setting it to true by default
        DeclareLaunchArgument(
            "use_qp_inria",
            default_value="true",
            description="Use QP solver from Inria",
        ),
        *declare_simulation_argument_list(
            robot_controller_config=robot_controller_config,
            robot_controller_config_path=robot_controller_config_path,
        ),
        *declare_hardware_argument_list(
            robot_controller_config=robot_controller_config,
            robot_controller_config_path=robot_controller_config_path,
        ),
    ]

    # --------------------------------------------------------------------------
    # Controllers spawner
    # --------------------------------------------------------------------------
    spawner_qontrol = declare_node_qcontrol_controller_spawner()

    spawner_sandbox_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sandbox_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_gripper_controller = declare_node_gripper_controller_spawner()

    robot_controller_list = [
        spawner_gripper_controller,
        spawner_qontrol,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawner_qontrol,
                on_exit=[spawner_sandbox_controller],
            )
        ),
    ]
    # --------------------------------------------------------------------------
    # File Paths & Substitutions
    # --------------------------------------------------------------------------
    # Config Files

    robot_simulation = declare_simulation_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=False,
    )

    robot_hardware = declare_hardware_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=False,
        robot_controller_config_file=robot_controller_config,
        robot_controller_config_path=robot_controller_config_path,
    )

    # --------------------------------------------------------------------------
    # Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        robot_simulation,
        robot_hardware,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)