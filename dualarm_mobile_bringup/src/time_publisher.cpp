#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "time_publisher");
	ros::NodeHandle nh;

	ros::Publisher time_pub = nh.advertise<rosgraph_msgs::Clock>("/rostime", 1000);

	ros::Rate loop_rate(1000);

	rosgraph_msgs::Clock msg;

	while(ros::ok())
	{
		msg.clock = ros::Time::now();
		time_pub.publish(msg);
	}

	return 0;

}
