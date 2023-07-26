""" A ROS2 node module for playing experimental task-related sounds. """

# Copyright 2022 Carnegie Mellon Neuromechatronics Lab
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from playsound import playsound
from rclpy import node
from rclpy.qos import QoSPresetProfiles
from example_interfaces.msg import String as state_message

DEFAULT_QOS = QoSPresetProfiles.SYSTEM_DEFAULT.value



class Node(node.Node):
  """
  A ROS2 node for playing experimental task-related sounds.
  
  This node listens for messages on a ROS2 topic designated by the 
  ``state_topic`` parameter. Each message consists of string data that 
  specifies the active state. When a message is received, the active state is 
  mapped to an audio file path -- via ROS2 parameters -- and the audio sample 
  is played aloud. If no parameter mapping exists for a given state, then no 
  action is taken.
  
  For simplicity, this node allows undeclared parameters. Therefore, new states 
  can be mapped to audio samples by simply adding the states to the loaded ROS2 
  parameters.
  
  Parameters
  ----------
  *args
      ``rclpy.node.Node`` arguments.
  node_name : str, default='audio'
      ROS2 node name.
  **kwargs
      ``rclpy.node.Node`` keyword arguments.
  
  Notes
  -----
  
  The mapping between state labels and audio filepaths can be specified in a 
  YAML parameters file as follows::
  
    /audio:
      ros__parameters:
        state_map:
          success: assets/success.wav
          intertrial: assets/trial.wav
  
  In this example, ``success.wav`` is played when the success condition is 
  achieved, and ``trial.wav`` is played when the task enters the intertrial 
  state.
  """
  
  def __init__(self, *args, node_name='audio', **kwargs):
      
      # Override the ROS default for this node, unless explicitly specified.
      kwargs = {'allow_undeclared_parameters': True,
                'automatically_declare_parameters_from_overrides':True,
                **kwargs}
      
      # Invoke the superclass constructor.
      super().__init__(*args, node_name=node_name, **kwargs)
      
      # Declare the topic parameter for the state subscription.
      #self.declare_parameter('state_topic', '/task/state')
      
      # Subscribe to the task state topic.
      kwargs \
        = dict(msg_type = state_message,
               topic = self.get_parameter('state_topic').value,
               callback = self.state_callback,
               qos_profile = DEFAULT_QOS)
      self.subscription = self.create_subscription(**kwargs)
      
      
  def state_callback(self, message, parameter_namespace='state_map'):
      """ ROS callback for task state messages.
      
      Parameters
      ----------
      message : ROS2 message
          Received ROS2 message. Contains the active state information.
      parameter_namespace : str, default='state_map'
          Parameter namespace. Prefix for parameter name.
      """
      kwargs = dict(state=message.data, parameter_namespace=parameter_namespace)
      self.play_state_transition_sound(**kwargs)
      
  def play_state_transition_sound(self, state, parameter_namespace='state_map'):
      """ Play an audio sample associated with a particular task state.
      
      Maps a given state to an audio sample (i.e., an audio file path) via 
      parameters. If a filepath parameter is not set for a provided state, then 
      this function takes no action.
      
      Parameters
      ----------
      state : str
          Name of the state associated with a sound.
      parameter_namespace : str, default='state_map'
          Parameter namespace. Prefix for parameter name.
      """
      parameter_name = f'{parameter_namespace}.{state}'
      audio_filepath = self.get_parameter_or(parameter_name, None).value
      if audio_filepath: playsound(audio_filepath)
      
      

