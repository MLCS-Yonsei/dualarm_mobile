#ifndef PUBLISHER_H
#define PUBLISHER_H
#include <stdint.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ethercat_test/vel.h>

#define PI 3.141592653589793238462643383279502884L

// Variables for computing odometry
double wheelRadius = 0.076;
double radps_to_rpm = 60.0 / 2.0 / PI;
double rpm_to_radps = 2.0 * PI / 60.0;
double gearRatio = 73.5;
double wheelSepearation = 0.3857;
double paramFKLinear = wheelRadius * rpm_to_radps / gearRatio / 4.0;
double paramFKAngular = paramFKLinear / wheelSepearation;
double paramIK = radps_to_rpm * gearRatio / wheelRadius;
double vel_lim = 0.74;


#endif
