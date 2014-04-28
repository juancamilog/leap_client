leap_client
====

ROS wrapper for the Leap Motion API.

Needs ROS hydro or newer, and the Leap SDK. It has only been tested on Ubuntu 14..04.

To compile, you need to set the LEAPSDK environment variable to the path where you have downloaded the Leap SDK.

To run, execute "roslaunch leap_client leap_client.launch"

To visualize the data run "rosrun rviz rviz" and load the rviz config file "launch/leap_client.rviz"
