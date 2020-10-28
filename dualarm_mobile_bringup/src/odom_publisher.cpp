#include <dualarm_mobile_bringup/publisher.h>


int rate;

bool broadcast_tf;

bool listen_tf;

std::string odom_topic;

std::string encoder_topic;

tf::StampedTransform transform;

nav_msgs::Odometry odom;

ethercat_test::vel encoder;

ros::Time currentTime;

double frontLeft = 0.0;
double frontRight = 0.0;
double rearRight = 0.0;
double rearLeft = 0.0;


void encoderCallback(const ethercat_test::vel& msg)
{
  FrontLeft  =  double(msg.velocity[0]);
  FrontRight = -double(msg.velocity[1]);
  RearRight  = -double(msg.velocity[2]);
  RearLeft   =  double(msg.velocity[3]);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_publisher");

  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  nh.param<int>("rate", rate, 200);
  nh.param<bool>("broadcast_tf", broadcast_tf, true);
  nh.param<bool>("listen_tf", listen_tf, false);
  nh.param<std::string>("odom_topic", odom_topic, "/odom");
  nh.param<std::string>("encoder_topic", encoder_topic, "/measure");

  ros::Rate loop_rate(rate);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
  ros::Subscriber encoder_sub = nh.subscribe(encoder_topic, 1, encoderCallback);

  transform.setIdentity();
  if (listen_tf)
  {
    listener.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(3.0));
  }

  while (ros::ok())
  {
    if (listen_tf)
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
    }
    else
    {
      if (!encoder_lock)
      {
        encoder_lock = true;
      }
      double linear_vel_x  = paramFKLinear  * (frontRight + frontLeft + rearRight + rearLeft);
      double linear_vel_y  = paramFKLinear  * (frontRight - frontLeft - rearRight + rearLeft);
      double angular_vel_z = paramFKAngular * (frontRight - frontLeft + rearRight - rearLeft);

      currentTime = ros::Time::now();

      double stepTime = currentTime.toSec() - odom.header.stamp.toSec();
      tf::Transform tf_from_vel;
      tf_from_vel.setIdentity();

      if ( abs(angular_vel_z) < 0.0001 )
      {
        odom_msg.pose.covariance[35] = 0.01;
        odom_msg.twist.covariance[35] = 0.01;

        tf_from_vel.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*stepTime), static_cast<double>(linear_vel_y*stepTime), 0.0));
      }
      else
      {
        odom_msg.pose.covariance[35] = 100.0;
        odom_msg.twist.covariance[35] = 100.0;

        //Follow circular arc
        double angleChange = angular_vel_z * stepTime;
        double arcRadius_x = linear_vel_x * stepTime / angleChange;
        double arcRadius_y = linear_vel_y * stepTime / angleChange;

        tf_from_vel.setOrigin(tf::Vector3(sin(angleChange) * arcRadius_x + cos(angleChange) * arcRadius_y - arcRadius_y,
                                  sin(angleChange) * arcRadius_y - cos(angleChange) * arcRadius_x + arcRadius_x,
                                  0.0));
        tf_from_vel.setRotation(tf::createQuaternionFromYaw(angleChange));
      }
      transform *= tf_from_vel;

      broadcaster.sendTransform(
        tf::StampedTransform(transform, currentTime, "odom", "base_footprint")
      );
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
