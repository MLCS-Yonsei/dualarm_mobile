#include <math.h>

#include "ros/ros.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

  
#define PI 3.14159265358979323846

//Odometry
//tf::Transform odom_transform;

/*
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, q);
  odom_transform = tf::Transform(q, tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0));
}
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "bring_publisher");
  ros::NodeHandle nh;

  boost::shared_ptr<ros::NodeHandle> rosnode;
  rosnode.reset(new ros::NodeHandle());

  //boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;
  //transform_broadcaster.reset(new tf::TransformBroadcaster());

  // Odometry message subscriber
  //ros::Subscriber odom_sub = nh.subscribe("/wheel_encoder/odom", 10, odomCallback);
  ros::Rate loop_rate(20);
  tf::TransformBroadcaster broadcaster;

  while(ros::ok())
  {
    ros::Time currentTime = ros::Time::now();

    /*
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
    */

    broadcaster.sendTransform(
    tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.42, 0.0, 0.66)),
    	currentTime,"odom", "camera_odom_frame"));

    broadcaster.sendTransform(
    tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.42, 0.0, -0.66)),
    	currentTime,"camera_pose_frame", "base_footprint"));

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
    	currentTime,"base_link", "imu_link"));

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

  } //End while (ros::ok())

  return 0;

}
