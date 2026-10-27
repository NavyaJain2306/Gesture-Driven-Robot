from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction  #used to start gazebo
from launch_ros.actions import Node
import os
def generate_launch_description():
    urdf_path = os.path.expanduser(
        '~/YOLO_MODEL/src/gesture_driven_robot/urdf/robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], #verbose - prints detailed output to the terminal(debugging)
            output='screen'
        ),

#robot_state_publisher — publishes TF and /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        #rviz (gazebo will read the urdf file directly, but rviz will take from here)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_wheel_tf',
            arguments=['0', '0.17', '-0.025', '0', '0', '0',
                       'base_link', 'left_wheel'],
            output='screen'
        ),

       
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right_wheel_tf',
            arguments=['0', '-0.17', '-0.025', '0', '0', '0',
                       'base_link', 'right_wheel'],
            output='screen'
        ),

        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-entity', 'gesture_robot',#the robot's name - gesture_robot
                '-file',   urdf_path,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),

    ])
