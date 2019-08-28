#include <math.h>

#include "ros/ros.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle nh;

  boost::shared_ptr<ros::NodeHandle> rosnode;
  rosnode.reset(new ros::NodeHandle());

  ros::Rate loop_rate(100);
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

  ros::Time lastTime = ros::Time::now();
  listener.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(10.0));
  listener.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(10.0));

  while(ros::ok())
  {
    tf::StampedTransform transform;
    geometry_msgs::Twist twist;
    nav_msgs::Odometry odom;
    ros::Time currentTime = ros::Time::now();
    try
    {
      listener.lookupTransform("odom", "base_footprint", currentTime, transform);
      listener.lookupTwist("odom", "base_footprint", currentTime, currentTime-lastTime, odom.twist.twist);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    tf::poseTFToMsg(transform, odom.pose.pose);

    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
		
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
		
    if (abs(odom.twist.twist.angular.z) < 0.0001) {
      odom.pose.covariance[35] = 0.01;
    }else{
      odom.pose.covariance[35] = 100.0;
    }

    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[14] = 0.001;
    odom.twist.covariance[21] = 1000000000000.0;
    odom.twist.covariance[28] = 1000000000000.0;
		
    if (abs(odom.twist.twist.angular.z) < 0.0001) {
      odom.twist.covariance[35] = 0.01;
    }else{
      odom.twist.covariance[35] = 100.0;
    }

    odom_pub.publish(odom);
    lastTime = currentTime;

    loop_rate.sleep();

  } //End while (ros::ok())

  return 0;

}
