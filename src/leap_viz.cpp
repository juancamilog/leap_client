#include "ros/ros.h"
#include "leap_client/HandInfoList.h"
#include "leap_client/HandInfo.h"
#include "leap_client/FingerInfo.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

static tf::TransformBroadcaster* br;
static ros::Publisher* marker_pub;

void leap_callback(const leap_client::HandInfoList::ConstPtr & msg){
    visualization_msgs::MarkerArray marker_array;
    //for each hand in the list
    for(unsigned int i=0; i<msg->hands.size() ; i++){
        leap_client::HandInfo hand = msg->hands[i];
        // publish the transform
        tf::Transform hand_transform;
        tf::Vector3 hand_origin = tf::Vector3(-hand.pose.position.z/1000,
                                              -hand.pose.position.x/1000,
                                               hand.pose.position.y/1000);
        tf::Quaternion hand_orientation = tf::Quaternion(-hand.pose.orientation.x,
                                                         -hand.pose.orientation.y,
                                                         -hand.pose.orientation.z,
                                                          hand.pose.orientation.w);
        hand_transform.setOrigin(hand_origin);
        hand_transform.setRotation(hand_orientation);
        br->sendTransform(tf::StampedTransform(hand_transform, hand.header.stamp, "leap", hand.header.frame_id));

        // create a hand marker
        visualization_msgs::Marker hand_marker;
        hand_marker.header.frame_id = hand.header.frame_id;
        hand_marker.header.stamp = hand.header.stamp;
        hand_marker.ns = "leap";
        hand_marker.id = hand.id;
        hand_marker.type= visualization_msgs::Marker::CUBE;
        hand_marker.action = visualization_msgs::Marker::ADD;
        hand_marker.scale.x = 0.1;
        hand_marker.scale.y = 0.07;
        hand_marker.scale.z = 0.02;
        hand_marker.color.r = 0.5f;
        hand_marker.color.g = 0.8f;
        hand_marker.color.b = 0.3f;
        hand_marker.color.a = 0.8f;
        hand_marker.lifetime = ros::Duration(0.1);

        marker_array.markers.push_back(hand_marker);
        // create a marker for the fingers
        visualization_msgs::Marker lines_marker;
        lines_marker.header.frame_id = "leap";
        lines_marker.header.stamp = hand.header.stamp;
        lines_marker.ns = "leap_lines";
        lines_marker.id = hand.id;
        lines_marker.type= visualization_msgs::Marker::LINE_LIST;
        lines_marker.action = visualization_msgs::Marker::ADD;
        lines_marker.scale.x = 0.02;
        lines_marker.scale.y = 0.02;
        lines_marker.scale.z = 0.02;
        lines_marker.color.r = 0.7f;
        lines_marker.color.g = 0.7f;
        lines_marker.color.b = 0.3f;
        lines_marker.color.a = 0.8f;
        lines_marker.lifetime = ros::Duration(0.1);

        for(unsigned int j=0; j<hand.fingers.size() ; j++){
            // publish a transform for the fingertips
            leap_client::FingerInfo finger = hand.fingers[j];
            tf::Transform finger_transform;
            tf::Vector3 tip_origin = tf::Vector3(-finger.tip_position.z/1000,
                                                 -finger.tip_position.x/1000,
                                                  finger.tip_position.y/1000);
            finger_transform.setOrigin(tip_origin);
            finger_transform.setRotation(tf::Quaternion(-hand.pose.orientation.x,
                                                        -hand.pose.orientation.y,
                                                        -hand.pose.orientation.z,
                                                        hand.pose.orientation.w));
            br->sendTransform(tf::StampedTransform(finger_transform, hand.header.stamp, "leap", finger.header.frame_id));

            // create the fingertip marker
            visualization_msgs::Marker finger_marker;
            finger_marker.header.frame_id = finger.header.frame_id;
            finger_marker.header.stamp = finger.header.stamp;
            finger_marker.ns = hand.header.frame_id;
            finger_marker.id = finger.id;
            finger_marker.type= visualization_msgs::Marker::CUBE;
            finger_marker.action = visualization_msgs::Marker::ADD;
            finger_marker.scale.x = 0.02;
            finger_marker.scale.y = 0.02;
            finger_marker.scale.z = 0.02;
            finger_marker.color.r = 0.7f;
            finger_marker.color.g = 0.7f;
            finger_marker.color.b = 0.3f;
            finger_marker.color.a = 0.8f;
            finger_marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(finger_marker);

            geometry_msgs::Point hand_point; 
            hand_point.x = hand_origin[0];
            hand_point.y = hand_origin[1];
            hand_point.z = hand_origin[2];
            geometry_msgs::Point tip_point; 
            tip_point.x = tip_origin[0];
            tip_point.y = tip_origin[1];
            tip_point.z = tip_origin[2];
            lines_marker.points.push_back(hand_point);
            lines_marker.points.push_back(tip_point);
        }
        marker_array.markers.push_back(lines_marker);
    }

    // publish the markers
    if(marker_array.markers.size()>0){
        marker_pub->publish(marker_array);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "leap_viz");
    ros::NodeHandle nh("~");

    br = new tf::TransformBroadcaster();

    std::string hand_info_topic;
    nh.param<std::string>("hands_topic", hand_info_topic, "/leap/hand_info");

    ros::Subscriber leap_sub = nh.subscribe<leap_client::HandInfoList>(hand_info_topic,1,leap_callback);
    ros::Publisher m_pub = nh.advertise<visualization_msgs::MarkerArray>("hand_markers",1);
    marker_pub = &m_pub;
    ros::spin();
    return 0;
}
