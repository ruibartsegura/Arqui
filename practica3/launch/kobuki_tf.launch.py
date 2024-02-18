# Copyright Rui Bartolome Segura
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    kobuki_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kobuki'),
            'launch',
            'kobuki.launch.py')),
        )

    tf_main_cmd = Node(
        package='practica3',
        executable='tf_main',
        name='tf_main',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ])

    ld = LaunchDescription()

    ld.add_action(tf_main_cmd)
    ld.add_action(kobuki_cmd)

    return ld
