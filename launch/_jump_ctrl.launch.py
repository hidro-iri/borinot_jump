from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    yaml_path_arg = DeclareLaunchArgument('yaml_path',
                                          description='Path of the yaml file with state publisher node parameters')
    robot_name_arg = DeclareLaunchArgument('robot_name', description='Robot name to load its corresponding URDF')
    platform_name_arg = DeclareLaunchArgument(
        'platform_name', description='Platform name of the UAM. To load the platform yaml file in eagle_mpc_lib')

    topic_platform_state = PythonExpression(["'/'+'", LaunchConfiguration('robot_name'), "'+'/platform/state'"])
    topic_arm_state = PythonExpression(["'/'+'", LaunchConfiguration('robot_name'), "'+'/arm/state'"])

    remappings = [
        ("platform_position", "/fmu/vehicle_local_position/out"),
        ("platform_attitude", "/fmu/vehicle_attitude/out"),
        ("platform_angular_velocity", "/fmu/vehicle_angular_velocity/out"),
        ("platform_control_mode", "/fmu/vehicle_control_mode/out"),
        ("platform_land_detected", "/fmu/vehicle_land_detected/out"),
        ("platform_direct_control", "/fmu/actuator_direct_control/in"),
        ("platform_vehicle_command", "/fmu/vehicle_command/in"),
        ("platform_battery_status", "/fmu/battery_status/out"),
        ("platform_state", topic_platform_state),
        ("arm_state", topic_arm_state),
        # Check what happens with arm topics when there is no arm
        ("odri_state", "/odri/robot_state"),
        ("arm_command", "/odri/robot_command"),
        ("arm_state_machine_status", "/odri/state_machine_status"),
        ("arm_service_state_transition", "/odri/robot_interface/state_transition")
    ]

    jump_ctrl = Node(package="borinot_jump",
                    executable="jump_ctrl",
                    name="jump_controller",
                    namespace=LaunchConfiguration('robot_name'),
                    output="both",
                    emulate_tty=True,
                    parameters=[
                        LaunchConfiguration('yaml_path'), {
                            "robot_name": LaunchConfiguration('robot_name')
                        }, {
                            "platform_name": LaunchConfiguration('platform_name')
                        }
                    ],
                    remappings=remappings,
                    # prefix=['lldb-server p --listen localhost:3000 --server --'],
                    # prefix=['lldb-server g localhost:3000 --'],
                    # prefix = ['xterm -e gdb -ex=r --args']
                    )

    ld = LaunchDescription()

    ld.add_action(yaml_path_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(platform_name_arg)
    ld.add_action(jump_ctrl)

    return ld
