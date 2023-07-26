"""
A ROS2 package for playing experimental task-related sound cues via ROS messages.

"""

# Copyright 2022 Carnegie Mellon Neuromechatronics Lab
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import rclpy
from .node import Node


def main(args=None):
    """ Default ROS2 entry point. """
    
    # Initialize ROS.
    rclpy.init()
    
    # Run a node by passing control to ROS.
    try:
        
        # Initialize the node.
        node = Node()
        
        # Spin the node.
        try: rclpy.spin(node)
        except KeyboardInterrupt: pass
        finally: node.destroy_node()
        
    # Shut ROS down.
    finally: rclpy.shutdown()
    
    # Return.
    return
        

