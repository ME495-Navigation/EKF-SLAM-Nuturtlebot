# loads turtlebot3_burger URDF into a robot_state_publisher
# optional viewing in RVIZ

# import necessary libraries
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage

def generate_launch_description():

    return LaunchDescription([
        # use_rviz argument
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true', 
            description='Choose whether or not to launch rviz',
            choices=['true','false']),
        # use_jsp argument
        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true', 
            description='Launch joint_state_publisher gui or not',
            choices=['true','false']),
        # setting color to purple
        DeclareLaunchArgument(
            name='color',
            default_value='purple',
            description='default color for the turtlebot',
            choices=['purple','red','green','blue','']),
        # rviz file launch config
        SetLaunchConfiguration(
            name='color_file',
            value=[TextSubstitution(text='basic_'),
                   LaunchConfiguration('color'),
                   TextSubstitution(text='.rviz')]),
        # joint_state_publisher executable
        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             condition=IfCondition(LaunchConfiguration('use_jsp')),
             namespace=LaunchConfiguration('color')),
        # robot_state_publisher executable
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             namespace=LaunchConfiguration('color'),
             parameters=[{'robot_description':
                          Command([ExecutableInPackage('xacro','xacro'),
                                   TextSubstitution(text=' '),
                                   PathJoinSubstitution([FindPackageShare('nuturtle_description'),
                                                         'urdf',
                                                         'turtlebot3_burger.urdf.xacro']),
                                                         TextSubstitution(text= ' color:='),
                                                         LaunchConfiguration('color')])},
                                   {'frame_prefix': [LaunchConfiguration('color'),'/']}
                                                         ]),
        # rviz executable
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nuturtle_description'),
                                                   'config',
                                                   LaunchConfiguration('color_file')]),
                        '-f', [LaunchConfiguration('color'),
                               TextSubstitution(text='/base_footprint')]],
            on_exit=Shutdown(),
            namespace=LaunchConfiguration('color')
        )
        
    ])

# Node(package='robot_state_publisher',
#              executable='robot_state_publisher',
#              parameters=[{'robot_description':
#                           ParameterValue(Command(['xacro ',
#                                                   LaunchConfiguration('xacro_file'),
#                                                   ' color:=',
#                                                   LaunchConfiguration('color')]),
#                                                   value_type=str)},
#                          {'frame_prefix': [LaunchConfiguration('color'),'/']}],
#             namespace=PathJoinSubstitution([LaunchConfiguration('color')])),


# namespace=PathJoinSubstitution([LaunchConfiguration(('color'))])