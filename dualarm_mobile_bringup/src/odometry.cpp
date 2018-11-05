#include <math.h>

#include "ros/ros.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
//#include <tf/transform_listner.h>

//#include "epos_tutorial/commendMsg.h"
//#include "mobile_control/motorsMsg.h"
//#include "epos_tutorial/realVel.h"



#define PI 3.14159265358979323846


//Motor speed in RPM - initialization
int64_t w0 = 0;
int64_t w1 = 0;
int64_t w2 = 0;
int64_t w3 = 0;



void measureCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{

	//w0 = msg->realVel[0];
	//w1 = msg->realVel[1];
	//w2 = msg->realVel[2];
	//w3 = msg->realVel[3];
	w0 = msg->data[0];
	w1 = msg->data[1];
	w2 = msg->data[2];
	w3 = msg->data[3];

}


tf::Transform getTransformForMotion(
	double linear_vel_x,
	double linear_vel_y,
	double angular_vel_z,
	double timeSeconds
)
{

	tf::Transform tmp;
	tmp.setIdentity();

	if (std::abs(angular_vel_z) < 0.0001){
		tmp.setOrigin(
			tf::Vector3(
				static_cast<double>(linear_vel_x*timeSeconds),
				static_cast<double>(linear_vel_y*timeSeconds),
				0.0
			)
		);
	}else{
		double distChange_x = linear_vel_x * timeSeconds;
		double distChange_y = linear_vel_y * timeSeconds;
		double angleChange = angular_vel_z * timeSeconds;

		double arcRadius_x = distChange_x / angleChange;
		double arcRadius_y = distChange_y / angleChange;

		tmp.setOrigin(
			tf::Vector3(
				std::sin(angleChange) * arcRadius_x + std::cos(angleChange) * arcRadius_y -arcRadius_y,
				std::sin(angleChange) * arcRadius_y + std::cos(angleChange) * arcRadius_x -arcRadius_x,
				0.0
			)
		);
		tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
	}
	return tmp;

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "odom_publisher");
	ros::NodeHandle nh;
	

	boost::shared_ptr<ros::NodeHandle> rosnode;
	rosnode.reset(new ros::NodeHandle());
	
	boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;
	transform_broadcaster.reset(new tf::TransformBroadcaster());

	
	ros::Subscriber measure_sub = nh.subscribe("/measure", 100, measureCallback);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
	
	tf::Transform odom_transform;
	odom_transform.setIdentity();

	ros::Rate loop_rate(50);
	
	ros::Time last_odom_publish_time = ros::Time::now();

	//Gear ratio
	int gear_ratio = 76;
	//radps_to_rpm : rad/sec --> rpm
	double radps_to_rpm = 60.0/2.0/PI;
	//rpm_to_radps : rpm --> rad/sec
	double rpm_to_radps = 2.0*PI/60;

	//Wheel specification in meter
	double wheel_diameter = 0.152;
	double wheel_radius = wheel_diameter/2.0;
	double wheel_separation_a = 0.2355;
	double wheel_separation_b = 0.281;
	double l = wheel_separation_a+wheel_separation_b;

	//Motor speed in rad/sec - initialization
	double wheel_speed_lf = 0;
	double wheel_speed_rf = 0;
	double wheel_speed_lb = 0;
	double wheel_speed_rb = 0;

	double linear_vel_x = 0;
	double linear_vel_y = 0;
	double angular_vel_z = 0;


	while(ros::ok())
	{

		nav_msgs::Odometry odom;

		ros::Time currentTime = ros::Time::now();

		wheel_speed_lf = (double) w0 * rpm_to_radps;
		wheel_speed_rf = (double) w1 * rpm_to_radps;
		wheel_speed_lb = (double) w2 * rpm_to_radps;
		wheel_speed_rb = (double) w3 * rpm_to_radps;

		linear_vel_x =
			wheel_radius/4.0*(wheel_speed_lf+wheel_speed_rf+wheel_speed_lb+wheel_speed_rb);

		linear_vel_y =
			wheel_radius/4.0*(-wheel_speed_lf+wheel_speed_rf+wheel_speed_lb-wheel_speed_rb);

		angular_vel_z =
			wheel_radius/(4.0*l)*(-wheel_speed_lf+wheel_speed_rf-wheel_speed_lb+wheel_speed_rb);

		double step_time = 0;
		step_time = currentTime.toSec() - last_odom_publish_time.toSec();
		last_odom_publish_time = currentTime;
		
		odom_transform =
			odom_transform*getTransformForMotion(
				linear_vel_x, linear_vel_y, angular_vel_z, step_time);

		tf::poseTFToMsg(odom_transform, odom.pose.pose);

		odom.twist.twist.angular.z = angular_vel_z;
		odom.twist.twist.linear.x  = linear_vel_x;
		odom.twist.twist.linear.y  = linear_vel_y;

		odom.header.stamp = currentTime;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		if (transform_broadcaster.get()){
			transform_broadcaster->sendTransform(
				tf::StampedTransform(
					odom_transform,
					currentTime,
					"odom",
					"base_footprint"
				)
			);
		}
		
		odom.pose.covariance[0] = 0.001;
		odom.pose.covariance[7] = 0.001;
		odom.pose.covariance[14] = 1000000000000.0;
		odom.pose.covariance[21] = 1000000000000.0;
		odom.pose.covariance[28] = 1000000000000.0;
		
		if (std::abs(angular_vel_z) < 0.0001) {
			odom.pose.covariance[35] = 0.01;
		}else{
			odom.pose.covariance[35] = 100.0;
		}

		odom.twist.covariance[0] = 0.001;
		odom.twist.covariance[7] = 0.001;
		odom.twist.covariance[14] = 0.001;
		odom.twist.covariance[21] = 1000000000000.0;
		odom.twist.covariance[28] = 1000000000000.0;
		
		if (std::abs(angular_vel_z) < 0.0001) {
			odom.twist.covariance[35] = 0.01;
		}else{
			odom.twist.covariance[35] = 100.0;
		}

		odom_pub.publish(odom);
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;

}
