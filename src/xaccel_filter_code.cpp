#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cmath>
#include <math.h>
#include <cstdlib>

using namespace message_filters;

#define QUEUESIZE 100
//TODO split class implementation and decleration
class xAccelFilter{
    public:
        xAccelFilter(ros::NodeHandle* nh);

        void accelRawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& amsg, const geometry_msgs::QuaternionStamped::ConstPtr& qmsg);
	
        void publishAccelAvg(void);

        void callbackPublishAccelAvg(const ros::TimerEvent&);

	double calcAccel(const geometry_msgs::QuaternionStamped orientMsg); 

	double angleDif(double angleA, double angleB);

	double doubleMod(double x, double y); 

        message_filters::Subscriber<geometry_msgs::Vector3Stamped> accelRawSub;
	message_filters::Subscriber<geometry_msgs::QuaternionStamped> quatRawSub;
      	ros::Publisher accelFilterPub;

       	TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::QuaternionStamped> sync;

        double x_samples[QUEUESIZE] = {0};
        double y_samples[QUEUESIZE] = {0};
        double x_avg;
        double y_avg;
        int i_x, i_y;

        double accel_avg;

};

void xAccelFilter::publishAccelAvg(void){
    std_msgs::Float64 msg;
    msg.data = accel_avg;
   // ROS_INFO("%lf | %lf", avg, msg.data);
    accelFilterPub.publish(msg);
}

void xAccelFilter::accelRawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& amsg, const geometry_msgs::QuaternionStamped::ConstPtr& qmsg) {
    //ROS_INFO("DEBUG");
    double x_ls, y_ls, x_ns, y_ns;
    
    geometry_msgs::Vector3Stamped accelData = *amsg;
    geometry_msgs::QuaternionStamped quatData = *qmsg;

    x_ls = x_samples[i_x];
    y_ls = y_samples[i_y];

    x_ns = accelData.vector.x;
    y_ns = accelData.vector.y;

    x_samples[i_x] = x_ns;
    y_samples[i_y] = y_ns;
    
    x_avg += x_ns / QUEUESIZE; // update average with new sample
    x_avg -= x_ls / QUEUESIZE; // due to a rolling average, the oldest sample must be removed from the average calculation

    y_avg += y_ns / QUEUESIZE; // update average with new sample
    y_avg -= y_ls / QUEUESIZE; // due to a rolling average, the oldest sample must be removed from the average calculation

    i_x = (i_x + 1) % QUEUESIZE; // idx goes from [0] to [QUEUESIZE - 1] with rollover
    i_y = (i_y + 1) % QUEUESIZE; // idx goes from [0] to [QUEUESIZE - 1] with rollover

    accel_avg = calcAccel(quatData); // find total linear acceleration (considering both x and y direction)

    publishAccelAvg();
}

double xAccelFilter::doubleMod(double x, double y) {
    	double myX = x;
	while(myX < 0) {
		myX = myX + y;
	}
	return myX - (int)(myX/y) * y;
}

double xAccelFilter::angleDif(double angleA, double angleB){
	double result;
	angleA = doubleMod(angleA, 360.0);
	angleB = doubleMod(angleB, 360.0);

	result = abs(angleA - angleB);

	if(360.0 - result < result){
		return 360.0 - result;
	}

	return result;
}
//const geometry_msgs::Vector3Stamped freeAccMsg, 
double xAccelFilter::calcAccel(const geometry_msgs::QuaternionStamped orientMsg) {
        double xAccel = x_avg;
        double yAccel = y_avg;


        #define PI 3.14159265
        const geometry_msgs::Quaternion q = orientMsg.quaternion;
        //double vehicleYaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z) * 180.0 / PI;
        double vehicleYaw = atan2(2.0*(q.w*q.z + q.x*q.y), (1 - 2.0*(q.y*q.y + q.z*q.z))) * 180.0 / PI;
	vehicleYaw = doubleMod(vehicleYaw, 360.0);
	// vehicle_yaw  --  number 0 to 360, 0 degrees is north (I believe anyway)
        
        // relative to north, remember positive x acceleration is going north
        // this is comparing the angle between the acceleration vector and the vector pointing north [ 1 0 ]
        double accYaw = doubleMod(acos((xAccel * 1.0 + yAccel * 0.0) / (sqrt(xAccel*xAccel + yAccel*yAccel)*sqrt(1.0*1.0 + 0.0*0.0))) * 180.0 / PI, 360.0);
	
	ROS_INFO("accYaw: %lf", accYaw);
        ROS_INFO("vehicleYaw: %lf", vehicleYaw);
	ROS_INFO("angleDif: %lf\n\n", angleDif(accYaw, vehicleYaw));
	//ROS_INFO("\n");	

        double accMag = sqrt(xAccel*xAccel + yAccel*yAccel);

        if(angleDif(accYaw, vehicleYaw) < 90.0) {
		//ROS_INFO("angle dif is less than 90.0\n");
                return accMag;
        } else {
                return -1.0*accMag;
        }
}


xAccelFilter::xAccelFilter(ros::NodeHandle* nh) :
  accelRawSub(*(nh), "filter/free_acceleration", 1),
  quatRawSub(*(nh), "filter/quaternion", 1),
  sync(accelRawSub, quatRawSub, 10) {
    i_x = 0;
    i_y = 0;
    x_avg = 0;
    y_avg = 0;
    //TODO setup sub/pub
    accelFilterPub = nh->advertise<std_msgs::Float64>("xAccelFilter/accel_x", 1);

    sync.registerCallback(boost::bind(&xAccelFilter::accelRawCallback, this, _1, _2));
    ROS_INFO("CONSTRUCTOR INIT");
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
