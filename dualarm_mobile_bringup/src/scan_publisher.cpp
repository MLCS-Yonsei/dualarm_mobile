#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

int64_t scan_msg = 0;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	fram_id=scan->header.frame_id
	angle_min=scan.angle_min;
    angle_max=scan.angle_max;
	angle_increment=scan.angle_increment;
	scan_time=scan.scan_time;
    time_increment=scan.time_increment;
    range_min=scan.range_min;
    range_max=scan.range_max;
	intensities=scan.intensities;
    ranges=scan.ranges;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_publisher");
	ros::NodeHandle nh;

	ros::Subscriber measure_sub = nh.subscribe("/scan", 100, scanCallback);
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_revised", 100);
	}
	
	scan_pub.publish(scan);
		ros::spinOnce();
		loop_rate.sleep();
	return 0;

}
