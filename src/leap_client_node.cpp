#include "LeapROSListener.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "leap_client");
    ros::NodeHandle nh("~");

    double refresh_rate;
    nh.param<double>("refresh_rate",refresh_rate,10.0);
    
    LeapROSListener listener(nh);
    Leap::Controller controller;
    ros::Rate loop_rate(refresh_rate);
    
    controller.addListener(listener);

    while(ros::ok()){
        listener.processFrame(controller.frame());
        ros::spinOnce();
        loop_rate.sleep();
    }
    controller.removeListener(listener);
    return 0;
}
