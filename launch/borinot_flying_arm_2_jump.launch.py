import sys

from launch_ros.actions import Node

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # BRINGUP CONFIGURATION
    platform_name = 'borinot'
    arm_name = 'flying_arm_2'
    robot_name = '_'.join([platform_name, arm_name])

    yaml_path = PathJoinSubstitution([FindPackageShare('borinot_jump'), 'config', robot_name, 'bringup.yaml'])

    bringup_args = {}

    ld = LaunchDescription()

    # ----- <JUMP CONTROL> --------
    bringup_args['jump_ctrl'] = {'yaml_path': yaml_path, 'robot_name': robot_name, 'platform_name': platform_name}

    jump_path = PathJoinSubstitution([FindPackageShare('borinot_jump'), 'launch', '_jump_ctrl.launch.py'])
    jump_src = PythonLaunchDescriptionSource([jump_path])
    jump_launch = IncludeLaunchDescription(jump_src, launch_arguments=bringup_args['jump_ctrl'].items())
    ld.add_action(jump_launch)
    # ----- </ JUMP CONTROL> --------

    # ----- <ACTUATOR CONTROL THROTTLE> --------
    throttle_act = Node(package="topic_tools",
                        executable="throttle",
                        name="actuator_thorttle",
                        output="both",
                        emulate_tty=True,
                        arguments=["messages", "/fmu/actuator_direct_control/in", "20"])

    ld.add_action(throttle_act)
    # ----- </ ACTUATOR CONTROL THROTTLE> --------

    return ld


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())