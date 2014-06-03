leap_client
====

ROS wrapper for the Leap Motion API.

Needs ROS hydro or newer, and the Leap SDK. It has only been tested on Ubuntu 14..04. You can get the Leap SDK here: https://developer.leapmotion.com/downloads

Once you have downloaded the SDK bundle, you need to set the LEAPSDK environment variable to the path where the LeapSDK folder exists. For example, on my system I added the line

```bash
    export LEAPSDK=~/lib/LeapSDK
```
to my ~/.bashrc file, because I copied the LeapSDK folder into ~/lib. The LeapSDK folder should contain the following files: **include/Leap.h**, **include/LeapMath.h**, **lib/x64/libLeap.so** and **lib/x86/libLeap.so**.


To run, execute "roslaunch leap_client leap_client.launch"

To visualize the data run "rosrun rviz rviz" and load the rviz config file "launch/leap_client.rviz"
