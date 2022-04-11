import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration#, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    ###### Defining launch configuration variables #######
    ip_address = LaunchConfiguration('ip_address')
    device_port = LaunchConfiguration('device_port')
    gripper_stroke = LaunchConfiguration('gripper_stroke')
    joint_state_topic = LaunchConfiguration('gripper_joint_state_topic')

    ###### Creating DeclareArgument actions ######
    ip_address_argument = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.6',
        description='IP address of the master IOL device')

    device_port_argument = DeclareLaunchArgument(
        'device_port',
        default_value='502',
        description='The port on the master IOL which the gripper is connected to')

    gripper_stroke_argument = DeclareLaunchArgument(
        'gripper_stroke',
        default_value='40.683074951171875',
        description='Stroke of the gripper in mm - from center to edge(half of the full value)')

    joint_state_topic_argument = DeclareLaunchArgument(
        'gripper_joint_state_topic',
        default_value='schunk/schunk_egh80/joint_states',
        description='The joint_state topic for the gripper joints which is used for the publisher')

    ###### Creating Node actions ######
    egh80_control_cmd = Node(
        package='schunk_control_egh80',
        executable='egh80_control_node',
        name='egh80_driver_node',
        output='screen',
        parameters=[
            {'ip_address': ip_address},
            {'device_port': device_port},
            {'gripper_stroke': gripper_stroke},
            {'gripper_joint_state_topic': joint_state_topic}

        ], #parses xacro into xml
        # arguments=[urdf_path]
    )
####################### Create the launch description and populate ####################### 
    ld = LaunchDescription()

    ###### Add declared arguments to the LaunchDescription ######
    ld.add_action(ip_address_argument)
    ld.add_action(device_port_argument)
    ld.add_action(gripper_stroke_argument)
    ld.add_action(joint_state_topic_argument)


    ###### Add nodes to the LaunchDescription ######
    ld.add_action(egh80_control_cmd)  #robot_state_publisher node

    return ld 