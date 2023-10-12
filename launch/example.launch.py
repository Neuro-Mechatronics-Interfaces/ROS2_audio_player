# Copyright 2023 Neuromechatronics Lab, Carnegie Mellon University
# 
# Created by: Jonathan Shulgach
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.




from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # Set the ROS2 domain space
    ros_domain = ExecuteProcess( cmd=["set ROS_DOMAIN_ID=42"], shell=True, output="screen" )

    audio_node = Node(package='nml_task_audio', 
                      executable='node',
                      #name='audio_node',
                      parameters=[{'config/audio.yaml'}],
                      )
    
    
    return LaunchDescription([ros_domain,
                              audio_node,
                             ])
    
