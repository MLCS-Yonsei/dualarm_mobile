#include <math.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>


#define PI 3.14159265358979323846

/* Declare Global Variables */
double linear_x_d;
double linear_y_d;
double angular_z_d;


// Callback function to subscribe //
void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    linear_x_d = cmd_vel->linear.x;
    linear_y_d = cmd_vel->linear.y;
    angular_z_d = cmd_vel->angular.z;
}


// Main function //
int main(int argc, char **argv)
{
  ros::init(argc,argv,"bringup_dual");
  ros::NodeHandle nh;

  //100 que size//
  ros::Publisher publ_input = nh.advertise<std_msgs::Int32MultiArray>("/input_msg",100);

  // Quesize : 100 //
  ros::Subscriber sub1 = nh.subscribe("/cmd_vel",100,cmdCallback);

  // Publish rate : 100Hz //
  ros::Rate loop_rate(100);

  double vel_linear;

  double u_x = 0;
  double u_y = 0;
  double u_p = 0;

// Linear velocity : 0.2m/s , dphidt constraint :  10 deg/sec ( 0.174 rad/sec ), and scale factor  //
  double v_lim       = 0.35;
  double dphidt_lim  = 0.35;

// Wheel specification in meter //
  double wheel_diameter = 0.152;
  double wheel_radius = wheel_diameter / 2.0;
  double wheel_separation_a = 0.2355;
  double wheel_separation_b = 0.281;
  double l = wheel_separation_a + wheel_separation_b;


// Motor speed in rad/sec - initialization //
  double wheel_speed_lf = 0;
  double wheel_speed_rf = 0;
  double wheel_speed_lb = 0;
  double wheel_speed_rb = 0;

// Gear ratio //

  int gear_ratio = 76;

// radps_to_rpm : rad/sec --> rpm //
// rpm_to_radps : rpm --> rad/sec //

  double radps_to_rpm = 60.0/2.0/PI;
  double rpm_to_radps = 2.0 * PI / 60;

// Motor speed in RPM - initialization //

  int w1 = 0;
  int w2 = 0;
  int w3 = 0;
  int w4 = 0;


  while(ros::ok())
  {
    
    std_msgs::Int32MultiArray input_msg;

	vel_linear = sqrt(linear_x_d*linear_x_d+linear_y_d*linear_y_d);

	if(vel_linear>v_lim)
    {
		u_p = v_lim * angular_z_d / vel_linear;
		u_x = v_lim * linear_x_d/ vel_linear;
		u_y = v_lim * linear_y_d/ vel_linear;
    }
	else
	{
		u_p = angular_z_d;
		u_x = linear_x_d;
		u_y = linear_y_d;
	}

    ROS_INFO("u_x : %lf u_y : %lf u_phi : %lf",u_x,u_y,u_p);


    // Inverse Kinematics for motor input (uint : RPM) //

    w1 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y - l * u_p);
    w2 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y + l * u_p);
    w3 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y - l * u_p);
    w4 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y + l * u_p);

    wheel_speed_lf = (double) w1 * rpm_to_radps;
    wheel_speed_rf = (double) w2 * rpm_to_radps;
    wheel_speed_lb = (double) w3 * rpm_to_radps;
    wheel_speed_rb = (double) w4 * rpm_to_radps;

    if(u_x * u_x + u_y * u_y + u_p * u_p < 0.01*0.01){
        w1 = 0;
        w2 = 0;
        w3 = 0;
        w4 = 0;

        u_x = 0;
        u_y = 0;
        u_p = 0;
    }

    input_msg.data.clear();
    input_msg.data.push_back(w1 * gear_ratio);
    input_msg.data.push_back(-w2 * gear_ratio);
    input_msg.data.push_back(w3 * gear_ratio);
    input_msg.data.push_back(-w4 * gear_ratio);

    publ_input.publish(input_msg);
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;

}
