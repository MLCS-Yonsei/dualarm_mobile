#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ethercat_test/vel.h>

#define PI 3.141592653589793238462643383279502884L


int rate;

bool broadcast_tf;

bool listen_tf;

std::string odom_topic;

std::string encoder_topic;

tf::StampedTransform transform;

nav_msgs::Odometry odom;

ethercat_test::vel encoder;

ros::Time currentTime;

double FrontLeft;
double FrontRight;
double RearRight;
double RearLeft;


tf::Transform getTransformForMotion(
  double linear_vel_x, double linear_vel_y, double angular_vel, double step_time)
{
  tf::Transform tmp;
  tmp.setIdentity();
  
  
  if (abs(angular_vel) < 0.0001)
  {
    //Drive straight
    tmp.setOrigin(
      tf::Vector3(
        static_cast<double>(linear_vel_x*step_time),
        static_cast<double>(linear_vel_y*step_time),
        0.0
      )
    );
    tmp.setRotation(tf::createQuaternionFromYaw(0.0));
  }
  else
  {
    //Follow circular arc
    double angleChange = angular_vel * step_time;
    double arcRadius_x = linear_vel_x * step_time / angleChange;
    double arcRadius_y = linear_vel_y * step_time / angleChange;
    
    tmp.setOrigin(
      tf::Vector3(
        sin(angleChange) * arcRadius_x + cos(angleChange) * arcRadius_y - arcRadius_y,
        sin(angleChange) * arcRadius_y - cos(angleChange) * arcRadius_x + arcRadius_x,
        0.0
      )
    );
    tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
  }
  
  return tmp;

} // end tf::Transform getTransformForMotion


nav_msgs::Odometry computeOdometry(
  double linear_vel_x, double linear_vel_y, double angular_vel_z, double step_time)
{
  nav_msgs::Odometry odom;

  transform *= getTransformForMotion(
    linear_vel_x, linear_vel_y, angular_vel_z, step_time
  );
  tf::poseTFToMsg(odom_transform, odom.pose.pose);

  odom.twist.twist.linear.x  = linear_vel_x;
  odom.twist.twist.linear.y  = linear_vel_y;
  odom.twist.twist.angular.z = angular_vel_z;

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.covariance[0] = 0.001;
  odom.pose.covariance[7] = 0.001;
  odom.pose.covariance[14] = 1000000000000.0;
  odom.pose.covariance[21] = 1000000000000.0;
  odom.pose.covariance[28] = 1000000000000.0;
  
  if ( abs(angular_vel_z) < 0.0001 )
  {
    odom.pose.covariance[35] = 0.01;
  }
  else
  {
    odom.pose.covariance[35] = 100.0;
  }

  odom.twist.covariance[0] = 0.001;
  odom.twist.covariance[7] = 0.001;
  odom.twist.covariance[14] = 0.001;
  odom.twist.covariance[21] = 1000000000000.0;
  odom.twist.covariance[28] = 1000000000000.0;

  if ( abs(angular_vel_z) < 0.0001 )
  {
    odom.twist.covariance[35] = 0.01;
  }
  else
  {
    odom.twist.covariance[35] = 100.0;
  }

  return odom;

} // end computeOdometry

#endif
