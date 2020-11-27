#include <dualarm_mobile_bringup/publisher.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>

int rate;

bool broadcast_tf;

bool listen_tf;

std::string odom_topic;

std::string encoder_topic;

tf::StampedTransform transform;

nav_msgs::Odometry odom;

ethercat_test::vel encoder;

ros::Time currentTime;

double linear_vel_x = 0.0;
double linear_vel_y = 0.0;
double angular_vel_z = 0.0;

bool isInitialized = false;
ros::Publisher rpm_pub;
ethercat_test::vel rpm_msg;
int smoothing_factor;

//int rpm_ref[4] = {0, 0, 0, 0};
//int rpm[4] = {0, 0, 0, 0};
TrajectoryPointMsg ref[2];
double startTime = 0.0;
double acc_lim = 0.005;


void trajCallback(const FeedbackMsg::ConstPtr& feedback)
{
  
  startTime = feedback->header.stamp.toSec();
  double t = ros::Time::now().toSec() - startTime;
  unsigned int selected_traj = feedback->selected_trajectory_idx;
  unsigned int end_idx = 1;
  while (t > feedback->trajectories[selected_traj].trajectory[end_idx].time_from_start.toSec())
    ++end_idx;
  }
  ref[0] = feedback->trajectories[selected_traj].trajectory[end_idx-1]; 
  ref[1] = feedback->trajectories[selected_traj].trajectory[end_idx];
  
}


void encoderCallback(const ethercat_test::vel& msg)
{

  double frontLeft  =  double(msg.velocity[0]);
  double frontRight = -double(msg.velocity[1]);
  double rearRight  = -double(msg.velocity[2]);
  double rearLeft   =  double(msg.velocity[3]);

  linear_vel_x  = paramFKLinear  * (frontRight + frontLeft + rearRight + rearLeft);
  linear_vel_y  = paramFKLinear  * (frontRight - frontLeft - rearRight + rearLeft);
  angular_vel_z = paramFKAngular * (frontRight - frontLeft + rearRight - rearLeft);

  double t = ros::Time::now().toSec() - startTime;
  double dt = ref[1].time_from_start.toSec() - ref[0].time_from_start.toSec();

  double a = t / dt;
  if (a < 0)
    a = 0.0;
  else if (a > 1)
    a = 1.0;
  double b = 1.0 - a;

  double u1 = a * ref[1].velocity.linear.x + b * ref[0].velocity.linear.x;
  double u2 = a * ref[1].velocity.linear.y + b * ref[0].velocity.linear.y;
  double u3 = a * ref[1].velocity.angular.z + b * ref[0].velocity.angular.z;
  double err_u1 = u1 - linear_vel_x;
  double err_u2 = u2 - linear_vel_y;
  double err_u3 = u3 - angular_vel_z;
  if (abs(err_u1) + abs(err_u2) + abs(err_u3) > acc_lim)
  {
    u1 = linear_vel_x  + acc_lim * err_u1;
    u2 = linear_vel_y  + acc_lim * err_u2;
    u3 = angular_vel_z + acc_lim * err_u3;
  }

  u3 *= wheelSepearation;
  double normDesired = abs(u1) + abs(u2) + abs(u3);
  if (normDesired > 0.0001)
  {
    if (normDesired > normLimit)
    {
      double scaleFactor = normLimit / normDesired;
      u1 *= scaleFactor;
      u2 *= scaleFactor;
      u3 *= scaleFactor;
    }
    rpm_msg.velocity[0] = int( paramIK * (u1 - u2 - u3));
    rpm_msg.velocity[1] = int(-paramIK * (u1 + u2 + u3));
    rpm_msg.velocity[2] = int(-paramIK * (u1 - u2 + u3));
    rpm_msg.velocity[3] = int( paramIK * (u1 + u2 - u3));
  }
  else
  {
    for (unsigned int idx = 0; idx < 4; ++idx)
      rpm_msg.velocity[idx] = 0;
  }

  if (isInitialized)
  {
    rpm_pub.publish(rpm_msg);
  }

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

  rpm_pub = nh.advertise<ethercat_test::vel>("/input_msg", 1);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
  ros::Subscriber encoder_sub = nh.subscribe(encoder_topic, 1, encoderCallback);
  ros::Subscriber traj_sub = nh.subscribe("/move_base/TebLocalPlannerROS/teb_feedback", 1, trajCallback);
  isInitialized = true;

  transform.setIdentity();
  if (listen_tf)
  {
    listener.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(3.0));
  }

  while (ros::ok())
  {
    ros::spinOnce();

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
      currentTime = ros::Time::now();

      double stepTime = currentTime.toSec() - odom.header.stamp.toSec();
      tf::Transform tf_from_vel;
      tf_from_vel.setIdentity();

      if ( abs(angular_vel_z) < 0.0001 )
      {
        odom.pose.covariance[35] = 0.01;
        odom.twist.covariance[35] = 0.01;

        tf_from_vel.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*stepTime), static_cast<double>(linear_vel_y*stepTime), 0.0));
      }
      else
      {
        odom.pose.covariance[35] = 100.0;
        odom.twist.covariance[35] = 100.0;

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
