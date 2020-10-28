#include <dualarm_mobile_bringup/publisher.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_publisher");

  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  nh.param<int>("rate", rate, 200);
  nh.param<std::string>("odom_topic", topic_name, "/odom");

  ros::Rate loop_rate(rate);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(topic_name, 10);

  listener.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(3.0));
  while (ros::ok())
  {

    try
    {
      currentTime = ros::Time::now();
      listener.waitForTransform("odom", "base_footprint", currentTime, ros::Duration(3.0));
      listener.lookupTransform("odom", "base_footprint", currentTime, transform);
      listener.lookupTwist("odom", "base_footprint", currentTime, ros::Duration(0.02), odom.twist.twist);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    tf::poseTFToMsg(transform, odom.pose.pose);

    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
		
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.0001;
    odom.pose.covariance[28] = 0.0001;
		
    if (abs(odom.twist.twist.angular.z) < 0.0001) {
      odom.pose.covariance[35] = 0.0001;
    }else{
      odom.pose.covariance[35] = 0.01;
    }

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;
    odom.twist.covariance[14] = 0.01;
    odom.twist.covariance[21] = 0.0001;
    odom.twist.covariance[28] = 0.0001;
		
    if (abs(odom.twist.twist.angular.z) < 0.0001) {
      odom.twist.covariance[35] = 0.0001;
    }else{
      odom.twist.covariance[35] = 0.01;
    }

    odom_pub.publish(odom);

    loop_rate.sleep();

  } // end while (ros::ok())

  return 0;

} // end main
