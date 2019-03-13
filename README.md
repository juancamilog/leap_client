leap_client
====

ROS wrapper for the Leap Motion API.

Needs ROS hydro or newer, and the **V2 Leap SDK**. It has been tested on Ubuntu 14..04 to 18.04 (ros hydro to melodic). You can get the **V2 Desktop SDK** here: https://developer.leapmotion.com/downloads

Once you have downloaded the SDK bundle, **you need to set the LEAPSDK environment variable to the path where the LeapSDK folder exists**. For example, on my system I added the line

```bash
    export LEAPSDK=~/lib/LeapSDK
```
to my ~/.bashrc file, because I copied the LeapSDK folder into ~/lib. The LeapSDK folder should contain the following files: **include/Leap.h**, **include/LeapMath.h**, **lib/x64/libLeap.so** and **lib/x86/libLeap.so**.


To run, execute "roslaunch leap_client leap_client.launch"

To visualize the data run ```rosrun rviz rviz``` and load the rviz config file ```launch/leap_client.rviz```. This can also be done by running
```
rosrun rviz rviz -d PATH_TO_YOUR_WORKSPACE/src/leap_client/launch/leap_client.rviz
```

In Ubuntu 15.04 or newer, you might need to start the leap driver manualy. You can do this by running

```
    sudo leapd
```
