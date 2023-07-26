
This ROS2 node is intended for the purpose of adding audio cues to an 
experimental task. Designed for experiments in the Neuromechatronics Lab at 
Carnegie Mellon University.

## Installation

This node requires the [playsound](https://github.com/TaylorSMarks/playsound#installation) Python package. 

The package is 
[built](https://docs.ros.org/en/galactic/Tutorials/Creating-Your-First-ROS2-Package.html#build-a-package)
in the manner standard for ROS2 packages.

## Parameters and configuration

The mapping between task states and audio samples is specified via ROS2 
parameters. All mapping parameters are contained within the ``state_map`` ROS2 
[parameter namespace](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.get_parameters_by_prefix). Each parameter name in 
this namespace should match the name of a state in the task state machine. The 
value of each parameter in this namespace should be a filesystem path to an 
audio file playable by the Python 
[playsound](https://pypi.org/project/playsound/) package.

To add new sounds, simply load a new parameter value. This node allows 
undeclared parameters, so sounds can be added dynamically, and without editing 
editing the source code.

### Example YAML parameter file

```
/audio:
  ros__parameters:
    state_map:
      success: assets/success.wav
      intertrial: assets/trial.wav
```

## Example usage

In a ROS2 terminal,[^ros_terminal] run the audio node with the sample parameter 
configuration file:

[^ros_terminal]: That is, a terminal for which ROS2 and the local ROS2 
workspace have been sourced.

```
ros2 run nml_task_audio audio --ros-args --params-file config/audio.yaml
```

In a separate ROS2 terminal, re-load the parameters file, to ensure that 
undeclared parameters are initialized:

```
ros2 param load /audio config/audio.yaml
```

Finally, simulate a state transition by publishing a message to the task state 
topic:

```
ros2 topic pub --once /task/state example_interfaces/msg/String "{data: success}"
```

The audio file associated with the success state should play.

