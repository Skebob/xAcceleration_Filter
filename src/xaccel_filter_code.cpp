#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "math.h"

//TODO split class declaration and implementaton
#define QUEUESIZE 100
class xAccelFilter{
    public:
        xAccelFilter(ros::NodeHandle* nh);

        void accelRawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

        void publishAccelAvg(void);

        void callbackPublishAccelAvg(const ros::TimerEvent&);

    private:
        ros::Subscriber accelRawSub;
        ros::Publisher accelFilterPub;

        double samples[QUEUESIZE] = {0};
        double avg;
        int idx;
};

void xAccelFilter::publishAccelAvg(void){
    std_msgs::Float64 msg;
    msg.data = avg;
    accelFilterPub.publish(msg);
}

void xAccelFilter::callbackPublishAccelAvg(const ros::TimerEvent&){
    std_msgs::Float64 msg;
    msg.data = avg;
    accelFilterPub.publish(msg);
}

void xAccelFilter::accelRawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    ROS_INFO("DEBUG");
    
    double lastSample = samples[idx];
    double x = msg->vector.x;
    //double y = msg->vector.y;
    //double newSample = sqrt((x*x)+(y*y)); // simple vector addition
    double newSample = 0.0;

    ROS_INFO("%lf", newSample);    

    samples[idx] = newSample;
    
    avg += newSample / QUEUESIZE; // update average with new sample
    avg -= lastSample / QUEUESIZE; // due to a rolling average, the oldest sample must be removed from the average calculation

    idx = (idx + 1) % QUEUESIZE; // idx goes from [0] to [QUEUESIZE - 1] with rollover

    publishAccelAvg();
}

xAccelFilter::xAccelFilter(ros::NodeHandle* nh){
    idx = 0;
    avg = 0;
    //TODO setup sub/pub
    ros::Subscriber accelRawSub = nh->subscribe<geometry_msgs::Vector3Stamped>("filter/free_acceleration", 1, &xAccelFilter::accelRawCallback, this);
    ros::Publisher accelFilterPub = nh->advertise<std_msgs::Float64>("xAccelFilter/accel_x", 1);
    ROS_INFO("CONSTRUCTOR INIT1");
    //ros::Timer timer1 = nh->createTimer(ros::Duration(0.01), publishAccelAvg, this);
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "xAccelFilter");
    ros::NodeHandle n;
    
    ros::AsyncSpinner spinner(1);

    ros::Time::waitForValid();

    xAccelFilter filter(&n);

    spinner.start();
    ROS_INFO("START");
    ros::waitForShutdown();   

    ROS_INFO("EXIT");
    return 0;
}
