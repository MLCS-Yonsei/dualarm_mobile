#include <dualarm_mobile_bringup/publisher.h>


bool isInitialized = false;
ros::Publisher rpm_pub;
ethercat_test::vel rpm_msg;

int smoothing_factor;
int smoothing_factor_slow;
int smoothing_threshold;
float desired_frequency;

int rpm_ref[4] = {0, 0, 0, 0};
//int rpm[4] = {0, 0, 0, 0};

double time_init = 0;
double cmd_last_time = 0;

void cmdCallback(const geometry_msgs::Twist& cmd_vel)
{
  for (unsigned int idx = 0; idx < 4; ++idx)
  {
      rpm_ref[idx] = 0;
  }

  double u1 = cmd_vel.linear.x;
  double u2 = cmd_vel.linear.y;
  double u3 = cmd_vel.angular.z;
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
    rpm_ref[0] = int( paramIK * (u1 - u2 - u3));
    rpm_ref[1] = int(-paramIK * (u1 + u2 + u3));
    rpm_ref[2] = int(-paramIK * (u1 - u2 + u3));
    rpm_ref[3] = int( paramIK * (u1 + u2 - u3));
  }

  cmd_last_time = ros::Time::now().toSec() - time_init;

//   if (isInitialized)
//   {
//     for (unsigned int idx = 0; idx < 4; ++idx)
//     {
//	     std::cout<<"smoothing f : "<<smoothing_factor<<std::endl;
//	     std::cout<<"rpm_ref["<<idx<<"] : "<<rpm_ref[idx]<<std::endl;
//	     std::cout<<"rpm["<<idx<<"] : "<<rpm[idx]<<std::endl;
//	     rpm_msg.velocity[idx] = (smoothing_factor * rpm_ref[idx] + (100 -smoothing_factor) * rpm[idx]) / 100;
//             std::cout<<"out["<<idx<<"] : "<<rpm_msg.velocity[idx]<<std::endl;
//     }
//     rpm_pub.publish(rpm_msg);
//   }
}

void encoderCallback(const ethercat_test::vel& msg)
{
  if (ros::Time::now().toSec() - time_init  - cmd_last_time > 0.3)
  {
      return;
  }

  int rpm[4] = {0, 0, 0, 0};

  rpm[0] =  msg.velocity[0];
  rpm[1] =  msg.velocity[1];
  rpm[2] =  msg.velocity[2];
  rpm[3] =  msg.velocity[3];

  std::cout<<"========================="<<std::endl;
  std::cout<<"[encoder] : "<<rpm[0]<<", "<<rpm[1]<<", "<<rpm[2]<<", "<<rpm[3]<<std::endl;
  std::cout<<"[cmd_Vel] : "<<rpm_ref[0]<<", "<<rpm_ref[1]<<", "<<rpm_ref[2]<<", "<<rpm_ref[3]<<std::endl;

  if (isInitialized)
  {
    // velocity sum
    int sum = abs(rpm_ref[0]) + abs(rpm_ref[1]) + abs(rpm_ref[2]) + abs(rpm_ref[3]);
    if (sum == 0)
    {
        for (unsigned int idx = 0; idx < 4; ++idx)
	{
	    rpm_msg.velocity[idx] = 0;
	}
    }
    else if (sum < smoothing_threshold)
    {
	// if cmd_vel is small value,  set smoothing_factor as more small
        for (unsigned int idx = 0; idx < 4; ++idx)
	{
	    rpm_msg.velocity[idx] = (smoothing_factor_slow * rpm_ref[idx] + (1000 - smoothing_factor_slow) * rpm[idx]) / 1000;
	}
    }
    else
    {
        for (unsigned int idx = 0; idx < 4; ++idx)
        {
	    rpm_msg.velocity[idx] = (smoothing_factor * rpm_ref[idx] + (1000 - smoothing_factor) * rpm[idx]) / 1000;
        }
    }
    
    std::cout<<"[output] : "<<rpm_msg.velocity[0]<<", "<<rpm_msg.velocity[1]<<", "<<rpm_msg.velocity[2]<<", "<<rpm_msg.velocity[3]<<std::endl;

    rpm_pub.publish(rpm_msg);
  }

  ros::Duration(1/desired_frequency).sleep();

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "cmd_vel_publisher");

  ros::NodeHandle nh;

  nh.param<int>("/cmd_vel_node/smoothing_factor", smoothing_factor, 40);
  nh.param<int>("/cmd_vel_node/smoothing_factor_slow", smoothing_factor_slow, 10);
  nh.param<int>("/cmd_vel_node/smoothing_threshold", smoothing_threshold, 150);
  nh.param<float>("/cmd_vel_node/desired_frequency", desired_frequency, 100);

  time_init = ros::Time::now().toSec();

  if (smoothing_factor > 100) {
    smoothing_factor = 100;
  }
  else if (smoothing_factor < 1) {
    smoothing_factor = 1;
  }
  if (smoothing_factor_slow > 100) {
    smoothing_factor_slow = 100;
  }
  else if (smoothing_factor_slow < 1) {
    smoothing_factor = 1;
  }

  rpm_pub = nh.advertise<ethercat_test::vel>("/input_msg", 1);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmdCallback);
  ros::Subscriber encoder_sub = nh.subscribe("/measure", 1, encoderCallback);
  isInitialized = true;
  ros::spin();

  return 0;

} // end main
