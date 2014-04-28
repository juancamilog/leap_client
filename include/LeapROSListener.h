#include "Leap.h"
#include "leap_client/HandInfoList.h"
#include "leap_client/HandInfo.h"
#include "leap_client/FingerInfo.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
using namespace Leap;

class LeapROSListener: public Listener {
    public:
        LeapROSListener(ros::NodeHandle &nh);
        virtual void onInit(const Controller&);
        virtual void onConnect(const Controller&);
        virtual void onDisconnect(const Controller&);
        virtual void onExit(const Controller&);
        virtual void processFrame(const Frame&);
    private:
        ros::Publisher hand_info_pub;
};

