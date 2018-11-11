#include <math.h>

#include "ros/ros.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265358979323846

//Motor
double linear_vel_x;
double linear_vel_y;
double angular_vel_z;

//LiDAR
sensor_msgs::LaserScan scan_ori;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	scan_ori.header.frame_id = scan->header.frame_id;
	scan_ori.angle_min = scan->angle_min;
	scan_ori.angle_max = scan->angle_max;
	scan_ori.angle_increment = scan->angle_increment;
	scan_ori.scan_time = scan->scan_time;
	scan_ori.time_increment = scan->time_increment;
	scan_ori.range_min = scan->range_min;
	scan_ori.range_max = scan->range_max;
	scan_ori.intensities = scan->intensities;
	scan_ori.ranges = scan->ranges;
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	linear_vel_x  = msg->linear.x;
	linear_vel_y  = msg->linear.y;
	angular_vel_z = msg->angular.z;
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

	ros::init(argc, argv, "bring_publisher");
	ros::NodeHandle nh;
	

	boost::shared_ptr<ros::NodeHandle> rosnode;
	rosnode.reset(new ros::NodeHandle());
	
	boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;
	transform_broadcaster.reset(new tf::TransformBroadcaster());

	//Odom
	ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 100, cmdCallback);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
	//LiDAR
	ros::Subscriber scan_sub = nh.subscribe("/scan_ori", 100, scanCallback);
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 100);
	
	tf::Transform odom_transform;
	odom_transform.setIdentity();

	ros::Rate loop_rate(100);
	
	ros::Time last_odom_publish_time = ros::Time::now();
	
	double wheel_separation_a = 0.2355;
	double wheel_separation_b = 0.281;
	double l = wheel_separation_a+wheel_separation_b;


	while(ros::ok())
	{
		ros::Time currentTime = ros::Time::now();

		nav_msgs::Odometry odom;
		sensor_msgs::LaserScan scan;
		tf::TransformBroadcaster broadcaster;

		double step_time = 0;
		step_time = currentTime.toSec() - last_odom_publish_time.toSec();
		last_odom_publish_time = currentTime;
		
		odom_transform =
			odom_transform*getTransformForMotion(
				linear_vel_x, linear_vel_y, angular_vel_z, step_time);

		tf::poseTFToMsg(odom_transform, odom.pose.pose);

		scan.header.stamp = currentTime;
		scan.header.frame_id = scan_ori.header.frame_id;
		scan.angle_min = scan_ori.angle_min;
		scan.angle_max = scan_ori.angle_max;
		scan.angle_increment = scan_ori.angle_increment;
		scan.scan_time = scan_ori.scan_time;
		scan.time_increment = scan_ori.time_increment;
		scan.range_min = scan_ori.range_min;
		scan.range_max = scan_ori.range_max;
		scan.intensities = scan_ori.intensities;
		scan.ranges = scan_ori.ranges;

		odom.twist.twist.linear.x  = linear_vel_x;
		odom.twist.twist.linear.y  = linear_vel_y;
		odom.twist.twist.angular.z = angular_vel_z;

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

		scan_pub.publish(scan);
		odom_pub.publish(odom);
		
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
			currentTime,"base_footprint", "base_link"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.281, -0.215, -0.020)),
			currentTime,"base_link", "br_wheel_link"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.281, 0.215, -0.020)),
			currentTime,"base_link", "bl_wheel_link"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.281, -0.215, -0.020)),
			currentTime,"base_link", "fr_wheel_link"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.281, 0.215, -0.020)),
			currentTime,"base_link", "fl_wheel_link"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
			currentTime,"base_footprint", "imu_link"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.57)),
			currentTime,"base_link", "base_scan"));
			
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, 0.0)), tf::Vector3(0.42, 0.0, 0.09)),
			currentTime,"base_link", "base_sonar_front"));
		
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, PI)), tf::Vector3(-0.41, 0.0, 0.09)),
			currentTime,"base_link", "base_sonar_rear"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, PI/2)), tf::Vector3(0.0, 0.3, 0.09)),
			currentTime,"base_link", "base_sonar_left"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, -PI/2)), tf::Vector3(0.0, -0.3, 0.09)),
			currentTime,"base_link", "base_sonar_right"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.41, 0.0, 0.465)),
			currentTime,"base_link", "base_kinect"));

		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.018, 0.0)),
			currentTime,"base_link", "kinect_depth_frame"));
	
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(-1.57079632679, 0.0, -1.57079632679)), tf::Vector3(0.0, 0.0, 0.0)),
			currentTime,"base_link", "kinect_depth_optical_frame"));
		
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, -0.005, 0.0)),
			currentTime,"base_link", "kinect_rgb_frame"));
		
		broadcaster.sendTransform(
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(-1.57079632679, 0.0, -1.57079632679)), tf::Vector3(0.0, 0.0, 0.0)),
			currentTime,"base_link", "kinect_rgb_optical_frame"));
		ros::spinOnce();
		loop_rate.sleep();
		
	}

	return 0;

}
