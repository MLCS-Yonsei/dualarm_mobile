#include <dualarm_mobile_bringup/publisher.h>


bool isInitialized = false;
ros::Publisher rpm_pub;
ethercat_test::vel rpm_msg;

void cmdCallback(const geometry_msgs::Twist& cmd_vel)
{
  int rpm_ref[4] = {0,0,0,0};

  double u1 = cmd_vel.linear.x;
  double u2 = cmd_vel.linear.y;
  double u3 = cmd_vel.angular.z;
  u3 *= wheelSepearation;

  double normDesired = abs(u1) + abs(u2) + abs(u3);

  if (normDesired > 0.0001)
  {
    if (normDesired > vel_lim)
    {
      double scaleFactor = vel_lim / normDesired;
      u1 *= scaleFactor;
      u2 *= scaleFactor;
      u3 *= scaleFactor;
    }
    rpm[0] = int( paramIK * (u1 - u2 - u3));
    rpm[1] = int(-paramIK * (u1 + u2 + u3));
    rpm[2] = int(-paramIK * (u1 - u2 + u3));
    rpm[3] = int( paramIK * (u1 + u2 - u3));
  }

   if (isInitialized)
   {
     for (unsigned int idx = 0; idx < 4; ++idx)
     {
	 rpm_msg.velocity[idx] = rpm_ref[idx];
     }
     rpm_pub.publish(rpm_msg);
   }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "cmd_vel_publisher");

  ros::NodeHandle nh;

  rpm_pub = nh.advertise<ethercat_test::vel>("/input_msg", 1);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel_teleop", 1, cmdCallback);

  isInitialized = true;
  ros::spin();

  return 0;

} // end main
