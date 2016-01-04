This code is intended as a minimal proof-of-concept for pushing ROS trajectoies
into HAL where they can be used to move motors.  There is intentionally very
little in the way of error checking or safety, to make the code both easier to
write and easier to understand.

It is assumed that ROS will be properly planning trajectories with appropriate
acceleration and velocity limits.  It is, however, fairly easy to get ROS to
generate discontinuous with large discontinuities.  If you plan to drive any
physical motors, it is recommended you use limit/limit2 comps to filter the
positions streamed from this HAL component.  You have been warned!

Minimal documentation:

Compile this project via ros:
catkin_make

Launch a ROS configuration that generates /joint_path_command messages
I used the Example 3.5 RViz:
http://aeswiki.datasys.swri.edu/rositraining/indigo/Exercises/3.5

Start the realtime environment and launch the HAL RT Ring:
realtime start
halcmd -f joint_stream.hal

Optional: Launch halscope to monitor joint states:
halscope -i ros.halscope &

Launch the python listener code:
rosrun listener listener.py

Optional: Launch halscope to monitor joint states:
halscope -i ros.halscope &

Send a trajectory path:
  Move the robot arm in RViz
  Click "Plan"
  Click "Execute"
  A message with the trajectory is sent
  Python listener pushes values into the ring buffer
  Real-time side does something with the data

ToDo:
x Integrate path filtering as per uniform_sample_filter:
-   Use orocos KDL (python bindings?)
+   Used simple linear interpolation, similar to industrial_robot_simulator
  Make code more intelligent, perhaps like:
    industrial_robot_simulator:
      https://github.com/ros-industrial/industrial_core/blob/indigo-devel/industrial_robot_simulator/industrial_robot_simulator#L361-L380
    joint_trajectory_streamer:
      https://github.com/ros-industrial/industrial_core/blob/indigo-devel/industrial_robot_client/src/joint_trajectory_streamer.cpp
  Integrate into ros_control?
    http://wiki.ros.org/ros_control#Overview

