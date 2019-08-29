#include <dualarm_mobile_bringup/publisher.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "bring_publisher");

  ros::NodeHandle nh;

  tf::TransformBroadcaster broadcaster;

  nh.param<int>("rate", rate, 30);

  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    ros::Time currentTime = ros::Time::now();

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
