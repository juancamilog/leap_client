#include "LeapROSListener.h"
#include <string>

LeapROSListener::LeapROSListener(ros::NodeHandle &nh){
    std::string hands_topic;
    nh.param<std::string>("hands_topic",hands_topic,"hand_info");
    hand_info_pub = nh.advertise<leap_client::HandInfoList>(hands_topic,1);
};

void LeapROSListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
};

void LeapROSListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
};

void LeapROSListener::onDisconnect(const Controller& controller) {
  std::cout << "Disconnected" << std::endl;
};

void LeapROSListener::onExit(const Controller& controller) {
  std::cout << "Finished" << std::endl;
};

void LeapROSListener::processFrame(const Frame &frame) {

    if (!frame.hands().isEmpty()) {
        ROS_INFO("Frame has %d hands!",frame.hands().count());
        ros::Time stamp = ros::Time::now();
        // We are going to publish all the hand information available
        std::vector<leap_client::HandInfo> hand_msg_list; 
        for(int i=0; i<frame.hands().count(); i++){
            const Hand hand = frame.hands()[i];
            if (hand.isValid()){
                ROS_INFO("Hand %d is valid",hand.id());
                leap_client::HandInfo hand_msg;
                // The Leap library reports all positions wrt to the leap motion device position
                hand_msg.header.frame_id = "hand"+std::to_string(static_cast<int>(hand.id()));
                hand_msg.header.stamp = stamp;

                hand_msg.id = static_cast<int>(hand.id());
                
                hand_msg.time_visible = hand.timeVisible();

                hand_msg.pose.position.x = hand.palmPosition().x;
                hand_msg.pose.position.y = hand.palmPosition().y;
                hand_msg.pose.position.z = hand.palmPosition().z;
                
                hand_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                        hand.palmNormal().roll(),hand.direction().pitch(),hand.direction().yaw());

                hand_msg.velocity.x = hand.palmVelocity().x;
                hand_msg.velocity.y = hand.palmVelocity().y;
                hand_msg.velocity.z = hand.palmVelocity().z;

                hand_msg.sphere_radius = hand.sphereRadius();
                hand_msg.sphere_center.x = hand.sphereCenter().x;
                hand_msg.sphere_center.y = hand.sphereCenter().y;
                hand_msg.sphere_center.z = hand.sphereCenter().z;

                hand_msg.stabilized_pose.position.x = hand.stabilizedPalmPosition().x;
                hand_msg.stabilized_pose.position.y = hand.stabilizedPalmPosition().y;
                hand_msg.stabilized_pose.position.z = hand.stabilizedPalmPosition().z;

                hand_msg.stabilized_pose.orientation = hand_msg.pose.orientation;

                // finger positions!
                std::vector<leap_client::FingerInfo> finger_msg_list; 
                const FingerList fingers = hand.fingers();
                if (!fingers.isEmpty()) {
                    for (int j = 0; j < fingers.count(); ++j) {
                        leap_client::FingerInfo finger_msg;
                        finger_msg.header.frame_id = "finger"+std::to_string(static_cast<int>(fingers[j].id()));
                        finger_msg.id = fingers[j].id();
                        finger_msg.header.stamp = stamp;
                        finger_msg.hand_id = hand_msg.id;
                        finger_msg.time_visible = fingers[j].timeVisible();
                        finger_msg.tip_position.x = fingers[j].tipPosition().x;
                        finger_msg.tip_position.y = fingers[j].tipPosition().y;
                        finger_msg.tip_position.z = fingers[j].tipPosition().z;
                        finger_msg.pointing_direction.x = fingers[j].direction().x;
                        finger_msg.pointing_direction.y = fingers[j].direction().y;
                        finger_msg.pointing_direction.z = fingers[j].direction().z;

                        finger_msg_list.push_back(finger_msg);
                    }

                }
                hand_msg.fingers = finger_msg_list;
                // publish the hand message
                hand_msg_list.push_back(hand_msg);
            }
        }
        leap_client::HandInfoList msg;
        msg.hands = hand_msg_list;
        hand_info_pub.publish(msg);
    }
};
