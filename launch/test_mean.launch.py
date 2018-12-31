#!/usr/bin/python3.6
#
# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import pathlib

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch_ros.actions


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'test_mean.yaml'
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
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
