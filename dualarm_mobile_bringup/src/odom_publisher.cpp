#include <dualarm_mobile_bringup/publisher.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>

//using teb_local_planner::TrajectoryPointMsg;
//using teb_local_planner::FeedbackMsg;

int rate;

bool broadcast_tf;

bool listen_tf;

std::string odom_topic;

std::string encoder_topic;

tf::StampedTransform transform;

nav_msgs::Odometry odom;

ethercat_test::vel encoder;

ros::Time currentTime;
ros::Time timeCmdReceived;
//ros::Time timeTeleopReceived;

double linear_vel_x = 0.0;
double linear_vel_y = 0.0;
double angular_vel_z = 0.0;

bool isInitialized = false;
ros::Publisher rpm_pub;
ethercat_test::vel rpm_msg;
//bool teleop_mode = false; 

double vel_ref[3] = {0.0, 0.0, 0.0};
//teb_local_planner::TrajectoryPointMsg ref[2];
double startTime = 0.0;
double acc_lim = 0.05;

double frontLeft;
double frontRight;
double rearRight;
double rearLeft;
int32_t sum_frontLeft;
int32_t sum_frontRight;
int32_t sum_rearRight;
int32_t sum_rearLeft;
int8_t encoder_count = 0;
bool encoder_lock = false;


//void trajCallback(const teb_local_planner::FeedbackMsg::ConstPtr& feedback)
//{
//
//  startTime = feedback->header.stamp.toSec();
//  double t = ros::Time::now().toSec() - startTime;
//  unsigned int selected_traj = feedback->selected_trajectory_idx;
//  unsigned int end_idx = 1;
//  while (t > feedback->trajectories[selected_traj].trajectory[end_idx].time_from_start.toSec())
//  {
//    ++end_idx;
//  }
//  ref[0] = feedback->trajectories[selected_traj].trajectory[end_idx-1]; 
//  ref[1] = feedback->trajectories[selected_traj].trajectory[end_idx];
//
//}


void cmdCallback(const geometry_msgs::Twist& cmd_vel)
{
  vel_ref[0] = cmd_vel.linear.x;
  vel_ref[1] = cmd_vel.linear.y;
  vel_ref[2] = cmd_vel.angular.z;
  timeCmdReceived = ros::Time::now();
}

//void teleopCallback(const geometry_msgs::Twist& cmd_vel_teleop)
//{
//  teleop_mode = true;
//  timeTeleopReceived = ros::Time::now();
//}


void encoderCallback(const ethercat_test::vel& msg)
{
  while (true)
  {
    if (!encoder_lock)
    {
      encoder_lock = true;
      ++encoder_count;
      sum_frontLeft  +=  msg.velocity[0];
      sum_frontRight += -msg.velocity[1];
      sum_rearRight  += -msg.velocity[2];
      sum_rearLeft   +=  msg.velocity[3];
      encoder_lock = false;
      break;
    }
  }
}


void computeVelocity()
{
  double denominator = 0;
  while (true)
  {
    if (!encoder_lock && encoder_count > 0)
    {
      encoder_lock = true;
      denominator = double(encoder_count);
      frontLeft  = double(sum_frontLeft);
      frontRight = double(sum_frontRight);
      rearRight  = double(sum_rearRight);
      rearLeft   = double(sum_rearLeft);
      encoder_lock = false;
      break;
    }
  }
  frontLeft  /= denominator;
  frontRight /= denominator;
  rearRight  /= denominator;
  rearLeft   /= denominator;
  linear_vel_x  = paramFKLinear  * frontRight + frontLeft + rearRight + rearLeft);
  linear_vel_y  = paramFKLinear  * frontRight - frontLeft - rearRight + rearLeft);
  angular_vel_z = paramFKAngular * frontRight - frontLeft + rearRight - rearLeft);
}


void publishCmdVel()
{
  double err_u1 = vel_ref[0] - linear_vel_x;
  double err_u2 = vel_ref[1] - linear_vel_y;
  double err_u3 = vel_ref[2] - angular_vel_z;
  err_u3 *= wheelSepearation;

  double normDesiredAcc = abs(err_u1) + abs(err_u2) + abs(err_u3);
  if (normDesiredAcc > acc_lim)
  { 
    double scaleFactorAcc = acc_lim / normDesiredAcc;
    err_u1 *= scaleFactorAcc;
    err_u2 *= scaleFactorAcc;
    err_u3 *= scaleFactorAcc;
  }

  double u1 = err_u1 + linear_vel_x;
  double u2 = err_u2 + linear_vel_y;
  double u3 = err_u3 + wheelSepearation * angular_vel_z;

  double normDesiredVel = abs(u1) + abs(u2) + abs(u3);
  if (normDesiredVel > 0.0001)
  {
    if (normDesiredVel > vel_lim)
    {
      double scaleFactorVel = vel_lim / normDesiredVel;
      u1 *= scaleFactorVel;
      u2 *= scaleFactorVel;
      u3 *= scaleFactorVel;
    }
    rpm_msg.velocity[0] = int32_t( paramIK * (u1 - u2 - u3));
    rpm_msg.velocity[1] = int32_t(-paramIK * (u1 + u2 + u3));
    rpm_msg.velocity[2] = int32_t(-paramIK * (u1 - u2 + u3));
    rpm_msg.velocity[3] = int32_t( paramIK * (u1 + u2 - u3));
  
  }
  else
  {
    for (unsigned int idx = 0; idx < 4; ++idx)
      rpm_msg.velocity[idx] = 0;
  }

  if (isInitialized)
  {
    std::cout<<"------------------------"<<std::endl;
    std::cout<<rpm_msg.velocity[0]<<std::endl;
    std::cout<<rpm_msg.velocity[1]<<std::endl;
    std::cout<<rpm_msg.velocity[2]<<std::endl;
    std::cout<<rpm_msg.velocity[3]<<std::endl;
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
  //nh.param<bool>("/odom_node/teleop_mode", teleop_mode, false);

  ros::Rate loop_rate(rate);

  rpm_pub = nh.advertise<ethercat_test::vel>("/input_msg", 1);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
  ros::Subscriber encoder_sub = nh.subscribe(encoder_topic, 1, encoderCallback);
  //ros::Subscriber traj_sub = nh.subscribe("/move_base/TebLocalPlannerROS/teb_feedback", 1, trajCallback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmdCallback);
  //ros::Subscriber telop_sub = nh.subscribe("/cmd_vel_teleop", 1, teleopCallback);
  isInitialized = true;

  //if (teleop_mode == true)
  //{
  //    ROS_INFO_STREAM("Teleop control mode. please check bringup.launch param");
  //}

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
      computeVelocity();

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
		
    if (abs(odom.twist.twist.angular.z) < 0.0001)
    {
      odom.pose.covariance[35] = 0.0001;
    }
    else
    {
      odom.pose.covariance[35] = 0.01;
    }

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;
    odom.twist.covariance[14] = 0.01;
    odom.twist.covariance[21] = 0.0001;
    odom.twist.covariance[28] = 0.0001;
		
    if (abs(odom.twist.twist.angular.z) < 0.0001)
    {
      odom.twist.covariance[35] = 0.0001;
    }
    else
    {
      odom.twist.covariance[35] = 0.01;
    }

    if (abs(odom.twist.twist.angular.z) < 0.0001)
    {
      odom.twist.covariance[35] = 0.0001;
    }
    else
    {
      odom.twist.covariance[35] = 0.01;
    }

    odom_pub.publish(odom);

    if (ros::Time::now().toSec() - timeCmdReceived.toSec() > 0.5)
    {
      std::cout<<"[INFO] no teb input(cmd_vel) input. motor fixed"<<std::endl;
    	for (unsigned int idx=0; idx<3; ++idx)
      {
	      vel_ref[idx] = 0;
      }
    }
    publishCmdVel();
//    if (ros::Time::now().toSec() - timeTeleopReceived.toSec() > 1) {
//	teleop_mode = false;
//	std::cout<<"[INFO] no teleop input(cmd_vel_teleop) for 1.0 sec. changed 'teleop_mode' to false"<<std::endl;
    //}

    loop_rate.sleep();

  } // end while (ros::ok())

  return 0;

} // end main
