#include "ros/ros.h"
#include "leap_client/HandInfo.h"
#include "leap_client/HandInfoList.h"

void leap_callback(const leap_client::HandInfoList::ConstPtr & msg){
    //for each hand in the list
    ROS_INFO_STREAM(">HANDS:"<<msg->hands.size());
    for(unsigned int i=0; i< msg->hands.size() ; i++){
        leap_client::HandInfo hand = msg->hands[i];
        ROS_INFO_STREAM(">>Hand "<<hand.id);
        ROS_INFO_STREAM(">>>sphere radius "<<hand.sphere_radius);
        ROS_INFO_STREAM(">>>sphere center "<<(hand.sphere_center.x-hand.pose.position.x)<<" "<<
                                             (hand.sphere_center.x-hand.pose.position.x)<<" "<<
                                             (hand.sphere_center.x-hand.pose.position.x));
        ROS_INFO_STREAM(">>>fingers "<<hand.fingers.size());
    }
    // TODO implement the geometry_msgs/Twist publishing here
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "leap_twist_controller");
    ros::NodeHandle nh("~");

    std::string hand_info_topic;
    nh.param<std::string>("hand_info_topic", hand_info_topic, "/leap/hand_info");

    ros::Subscriber leap_sub = nh.subscribe<leap_client::HandInfoList>(hand_info_topic,1,leap_callback);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate r(50);
    while ( ros::ok() ){
        r.sleep();
    }
    return 0;
}
