Minimal documentation:

Compile this project via ros:
catkin_make

Launch a ROS configuration that generates /joint_path_command messages
I used the Example 3.5 RViz:
http://aeswiki.datasys.swri.edu/rositraining/indigo/Exercises/3.5

From machinekit/src/machinetalk/msgcomponents launch the HAL RT Ring:
halcmd -f recordread.hal

Launch the python listener code:
rosrun listener listener.py

Send a trajectory path:
  Move the robot arm in RViz
  Click "Plan"
  Click "Execute"
  A message with the trajectory is sent
  Python listener pushes values into the ring buffer
  Real-time side does something with the data

ToDo:
  Craft real-time component to glue float value(s) to HAL pin(s)
  Integrate path filtering as per uniform_sample_filter:
    Use orocos KDL (python bindings?)
  Make code more intelligent, perhaps like:
    industrial_robot_simulator:
      https://github.com/ros-industrial/industrial_core/blob/indigo-devel/industrial_robot_simulator/industrial_robot_simulator#L361-L380
    joint_trajectory_streamer:
      https://github.com/ros-industrial/industrial_core/blob/indigo-devel/industrial_robot_client/src/joint_trajectory_streamer.cpp
  Integrate into ros_control?
    http://wiki.ros.org/ros_control#Overview

