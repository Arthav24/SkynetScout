# Copyright 2024 SkynetScout
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
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument(
        "enableCrackDetection", description="Enable Crack Detection",
        default_value=TextSubstitution(text="true")
    ), DeclareLaunchArgument(
        "enableBeamDetection", description="Enable Beam Detection",
        default_value=TextSubstitution(text="true")
    ), DeclareLaunchArgument(
        "enableObjectDetection", description="Enable Hazardous Object Detection",
        default_value=TextSubstitution(text="true")
    )]

    talker_node = Node(
        package='anomaly_detection',
        executable='anomaly_detection_node',
        name='anomaly_detection_node',
        output='screen',
        emulate_tty=True,
        arguments=[],
        parameters=[{
            "enableCrackDetection": LaunchConfiguration('enableCrackDetection'),
            "enableBeamDetection": LaunchConfiguration('enableBeamDetection'),
            "enableObjectDetection": LaunchConfiguration('enableObjectDetection')
        }]
    )
    return LaunchDescription([
        *args,
        talker_node
    ])
