import sys

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

    # ----- <PLATFORM STATE ESTIMATION> --------

    # STATIC TRANSFORMS
    st_path = PathJoinSubstitution([FindPackageShare('eagle_ros2_tf'), 'launch', '_static_transforms.launch.py'])
    st_src = PythonLaunchDescriptionSource([st_path])
    st_launch = IncludeLaunchDescription(st_src)
    ld.add_action(st_launch)

    # MICRORTPS AGENT
    bringup_args['rtps'] = {'experiment_type': 'robot', 'robot_ip': '192.168.0.3'}

    rtps_path = PathJoinSubstitution([FindPackageShare('eagle_ros2_control'), 'launch', '_micrortps_agent.launch.py'])
    rtps_src = PythonLaunchDescriptionSource([rtps_path])
    rtps_launch = IncludeLaunchDescription(rtps_src, launch_arguments=bringup_args['rtps'].items())
    ld.add_action(rtps_launch)

    # VRPN CLIENT
    bringup_args['vrpn'] = {'yaml_path': yaml_path}

    vrpn_path = PathJoinSubstitution([FindPackageShare('eagle_ros2_tf'), 'launch', '_vrpn_client.launch.py'])
    vrpn_src = PythonLaunchDescriptionSource([vrpn_path])
    vrpn_launch = IncludeLaunchDescription(vrpn_src, launch_arguments=bringup_args['vrpn'].items())
    ld.add_action(vrpn_launch)

    # VISION POSE
    bringup_args['vision_pose'] = {'yaml_path': yaml_path}

    vision_pose_path = PathJoinSubstitution([FindPackageShare('eagle_ros2_tf'), 'launch', '_vision_pose.launch.py'])
    vision_pose_src = PythonLaunchDescriptionSource([vision_pose_path])
    vision_pose_launch = IncludeLaunchDescription(vision_pose_src,
                                                  launch_arguments=bringup_args['vision_pose'].items())
    ld.add_action(vision_pose_launch)

    # PLATFORM STATE PUBLISHER
    bringup_args['state_pub'] = {'yaml_path': yaml_path, 'robot_name': robot_name}

    state_pub_path = PathJoinSubstitution([FindPackageShare('eagle_ros2_tf'), 'launch', '_state_pub_sub.launch.py'])
    state_pub_src = PythonLaunchDescriptionSource([state_pub_path])
    state_pub_launch = IncludeLaunchDescription(state_pub_src, launch_arguments=bringup_args['state_pub'].items())
    ld.add_action(state_pub_launch)
    # ----- </ PLATFORM STATE ESTIMATION> --------

    # ----- <ARM INTERFACE> --------
    bringup_args['odri_iface'] = {'yaml_path': yaml_path, 'robot_name': arm_name}

    odri_iface_path = PathJoinSubstitution(
        [FindPackageShare('odri_interface'), 'launch', '_robot_interface.launch.py'])
    odri_iface_src = PythonLaunchDescriptionSource([odri_iface_path])
    odri_iface_launch = IncludeLaunchDescription(odri_iface_src, launch_arguments=bringup_args['odri_iface'].items())
    ld.add_action(odri_iface_launch)
    # ----- </ ARM STATE ESTIMATION> --------

    # ----- <CONTROL WATCHDOG> --------
    wdog_path = PathJoinSubstitution([FindPackageShare('eagle_ros2_control'), 'launch', '_watchdog_mc.launch.py'])
    wdog_src = PythonLaunchDescriptionSource([wdog_path])
    wdog_launch = IncludeLaunchDescription(wdog_src)
    ld.add_action(wdog_launch)
    # ----- </ CONTROL WATCHDOG> --------

    return ld


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())