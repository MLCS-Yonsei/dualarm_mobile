#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  
  double pi = 3.1415926535897932385;

  while(n.ok())
  {
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_footprint", "base_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.281, -0.215, -0.020)),
        ros::Time::now(),"base_link", "br_wheel_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.281, 0.215, -0.020)),
        ros::Time::now(),"base_link", "bl_wheel_joint"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.281, -0.215, -0.020)),
        ros::Time::now(),"base_link", "fr_wheel_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.281, 0.215, -0.020)),
        ros::Time::now(),"base_link", "fl_wheel_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_link", "imu_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.57)),
        ros::Time::now(),"base_link", "base_scan"));
        
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, 0.0)), tf::Vector3(0.42, 0.0, 0.09)),
        ros::Time::now(),"base_link", "base_sonar_front"));
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, pi)), tf::Vector3(-0.41, 0.0, 0.09)),
        ros::Time::now(),"base_link", "base_sonar_rear"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, pi/2)), tf::Vector3(0.0, 0.3, 0.09)),
        ros::Time::now(),"base_link", "base_sonar_left"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, -pi/2)), tf::Vector3(0.0, -0.3, 0.09)),
        ros::Time::now(),"base_link", "base_sonar_right"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.41, 0.0, 0.465)),
        ros::Time::now(),"base_link", "base_kinect"));

    r.sleep();
  }
}