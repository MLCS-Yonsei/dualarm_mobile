#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan scan_ori;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	scan_ori.header.frame_id = scan->header.frame_id;
	scan_ori.angle_min = scan->angle_min;
	scan_ori.angle_max = scan->angle_max;
	scan_ori.angle_increment = scan->angle_increment;
	scan_ori.scan_time = scan->scan_time;
	scan_ori.time_increment = scan->time_increment;
	scan_ori.range_min = scan->range_min;
	scan_ori.range_max = scan->range_max;
	scan_ori.intensities = scan->intensities;
	scan_ori.ranges = scan->ranges;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_publisher");
	ros::NodeHandle nh;

	ros::Subscriber measure_sub = nh.subscribe("/scan_ori", 100, scanCallback);
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 100);
	// ros::Rate loop_rate(50);
	while(ros::ok())
	{
		sensor_msgs::LaserScan scan_revised;
		scan_revised.header.stamp = ros::Time::now();
		scan_revised.header.frame_id = scan_ori.header.frame_id;
		scan_revised.angle_min = scan_ori.angle_min;
		scan_revised.angle_max = scan_ori.angle_max;
		scan_revised.angle_increment = scan_ori.angle_increment;
		scan_revised.scan_time = scan_ori.scan_time;
		scan_revised.time_increment = scan_ori.time_increment;
		scan_revised.range_min = scan_ori.range_min;
		scan_revised.range_max = scan_ori.range_max;
		scan_revised.intensities = scan_ori.intensities;
		scan_revised.ranges = scan_ori.ranges;
		scan_pub.publish(scan_revised);
		ros::spinOnce();
		// loop_rate.sleep();
	}
	return 0;
}
