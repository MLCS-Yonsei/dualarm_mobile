#include <math.h>
#include "ros/ros.h"
//#include "bringup_dual/commendMsg.h"
//#include "mobile_control/motorsMsg.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>


#define PI 3.14159265358979323846

/* Declare Global Variables */

// From commend_msg //
double pos_des[3];

// From odometry msg //
double pos_act[2];
double vel_act[3];
double quat_act[4];



// Callback function to subscribe //

void cmdCallback(const geometry_msgs::Pose2D::ConstPtr& cmd_msg)
{
    pos_des[0]   = cmd_msg->x;
    pos_des[1]   = cmd_msg->y;
    pos_des[2]   = cmd_msg->theta;
}
//void cmdCallback(const bringup_dual::commendMsg::ConstPtr& cmd_msg)
//{
    //pos_des[0]   = cmd_msg->xd;
    //pos_des[1]   = cmd_msg->yd;
    //pos_des[2]   = cmd_msg->phid;
//}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pos_act[0] = msg->pose.pose.position.x;
    pos_act[1] = msg->pose.pose.position.y;

    vel_act[0]  = msg->twist.twist.linear.x;
    vel_act[1]  = msg->twist.twist.linear.y;
    vel_act[2]  = msg->twist.twist.angular.z;

    quat_act[0] = msg->pose.pose.orientation.x;
    quat_act[1] = msg->pose.pose.orientation.y;
    quat_act[2] = msg->pose.pose.orientation.z;
    quat_act[3] = msg->pose.pose.orientation.w;
}

// Main function //

int main(int argc, char **argv)
{
  ros::init(argc,argv,"bringup_dual");
  ros::NodeHandle nh;

  //100 que size//
  ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);
  ros::Publisher publ_input = nh.advertise<std_msgs::Int32MultiArray>("/input_msg",100);

  // Quesize : 100 //
  ros::Subscriber sub1 = nh.subscribe("/ns1/cmd_msg",100,cmdCallback);
  ros::Subscriber sub2 = nh.subscribe("/odom",100,odomCallback);

  // Publish rate : 50Hz //
  ros::Rate loop_rate(50);

  // time unit : sec & loop time = 1/50 sec (50Hz) //
  double dt = 0.02;


//** Initialization **//
  // reference {prev, current} //
  double x_d[2]      = {0, 0};
  double y_d[2]      = {0, 0};
  double phi_d[2]    = {0, 0};
  
  // From Odometry //
  double x_act[2]    = {0, 0};
  double y_act[2]    = {0, 0};
  double phi_act[2]  = {0, 0};

  double vx_act[2]   = {0, 0};
  double vy_act[2]   = {0, 0};
  double dphi_act[2] = {0, 0};

  double x_quat   = 0;
  double y_quat   = 0;
  double z_quat   = 0;
  double w_quat   = 0;

  double angle = 0;

//** Controller gains Setting **//

  // P control //
  double kp_s = 1.0;
  double kp_phi = 10.0;

  // D control //
  double kd_s = 0.0; 
  double kd_phi = 0.0;

  // I control //
  double ki_s = 0.0;
  double ki_phi = 0.0;

  // accumulated error //
  double Is = 0.0;
  double Iphi = 0.0;

// Input(Velocity) //

  double del_s[2]={0,0};
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
    
    geometry_msgs::Twist cmd_vel;
    //bringup_dual::motorsMsg input_msg;
    std_msgs::Int32MultiArray input_msg;

/*
    epos_tutorial::DesiredVel input_msg;
*/
    // Current values from reference  phi --> rad//
    x_d[1]      = pos_des[0];
    y_d[1]      = pos_des[1];
    phi_d[1]    = pos_des[2];

    // Current values from Odometry  //
    
    x_act[1]    = pos_act[0];
    y_act[1]    = pos_act[1];

    vx_act[1]   = vel_act[0];
    vy_act[1]   = vel_act[1];
    dphi_act[1] = vel_act[2];

    x_quat   = quat_act[0];
    y_quat   = quat_act[1];
    z_quat   = quat_act[2];
    w_quat   = quat_act[3];
    
     
    
    /** Calculation of angles **/
    phi_act[1]  = atan2(2.0 * (w_quat * z_quat + x_quat * y_quat),
                        1.0-2.0 * (y_quat*y_quat + z_quat * z_quat));

    angle = atan2(y_d[1]-y_act[1],x_d[1]-x_act[1]);


   /* PID control */

  del_s[1] = sqrt( (x_d[1] - x_act[1]) * (x_d[1] - x_act[1]) + (y_d[1] - y_act[1]) * (y_d[1] - y_act[1]));
  
  vel_linear = kp_s * del_s[1] + kd_s * ( del_s[1] - del_s[0] ) / dt + ki_s * Is;

  u_p =  kp_phi * (phi_d[1]-phi_act[1])
        +kd_phi * ((phi_d[1] - phi_d[0])-(phi_act[1] - phi_act[0]))/dt
        +ki_phi * Iphi;

  


    if(vel_linear>v_lim)
    {
        vel_linear = v_lim;

    }

    if(dphidt_lim < u_p)
    {
      u_p = dphidt_lim;
    }
    else if(-dphidt_lim > u_p)
    {
      u_p = -dphidt_lim;

    }

    u_x = vel_linear * cosf(angle-phi_act[1]);
    u_y = vel_linear * sinf(angle-phi_act[1]);
    

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

    // Forward Kinematics //
    u_x = wheel_radius / 4.0 * (wheel_speed_lf + wheel_speed_rf + wheel_speed_lb + wheel_speed_rb);

    u_y = wheel_radius / 4.0 * (-wheel_speed_lf + wheel_speed_rf + wheel_speed_lb - wheel_speed_rb);

    u_p = wheel_radius / ( 4.0 * l) * (-wheel_speed_lf + wheel_speed_rf -wheel_speed_lb + wheel_speed_rb);


    cmd_vel.linear.x  = u_x;
    cmd_vel.linear.y  = u_y;
    cmd_vel.angular.z = u_p;

    input_msg.data.clear();
    input_msg.data.push_back(w1 * gear_ratio);
    input_msg.data.push_back(-w2 * gear_ratio);
    input_msg.data.push_back(w3 * gear_ratio);
    input_msg.data.push_back(-w4 * gear_ratio);
    //input_msg.omega1 = w1 * gear_ratio;
    //input_msg.omega2 = -w2 * gear_ratio;
    //input_msg.omega3 = w3 * gear_ratio;
    //input_msg.omega4 = -w4 * gear_ratio;
/*
    input_msg.vel1 = w1;
    input_msg.vel2 = w2;
    input_msg.vel3 = w3;
    input_msg.vel4 = w4;
*/
    ctrl_pub.publish(cmd_vel);
    publ_input.publish(input_msg);
    ros::spinOnce();
    loop_rate.sleep();

        // Last values from reference  phi --> rad//
    x_d[0]      = x_d[1];
    y_d[0]      = y_d[1];
    del_s[0]    = del_s[1];
    phi_d[0]    = phi_d[1];

    // Last values from Odometry  //
    
    x_act[0]    = x_act[1];
    y_act[0]    = y_act[1];
    phi_act[0]  = phi_act[1];
  
    vx_act[0]   = vx_act[1];
    vy_act[0]   = vy_act[1];
    dphi_act[0] = dphi_act[1];

    // Accumulated error update //
    Is = Is + (del_s[1]-del_s[0])*dt;
    Iphi = Iphi + (phi_d[1] - phi_act[1])*dt;

  }

  return 0;

}
