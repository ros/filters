from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir /'test_mean.yaml'
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription ([
            launch_ros.actions.Node(
            package='filters', node_executable='test_mean',
            output='screen',
	    parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_mean.yaml'],
           ],
           ),
])
