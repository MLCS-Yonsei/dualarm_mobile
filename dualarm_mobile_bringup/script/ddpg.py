#!/usr/bin/env python

#--------Include modules---------------
import os
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from tf import TransformListener
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from time import time
import numpy as np
from actor_net import ActorNet

# Subscribers' callbacks----------------------------------------
mapData=OccupancyGrid()
scandata=[]
path=[]


# def mapCallBack(data):
#     global mapData
#     mapData=data
    
def scanCallBack(data):
	global scandata   
	scandata=list(data.ranges)
	i=0
	while i < len(scandata):
		if scandata[i] == 0 :
			scandata[i] = 10.0
		i=i+1

def pathCallback(data):
	global path
	path=[]
	for pose in data.poses:
		path.append(list([pose.pose.position.x,pose.pose.position.y,euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w])[2]]))
	
# Node------------------------------------------------------------------

def node():
	global scandata,lin_vel_t,ang_vel_t,w
	rospy.init_node('ddpg', anonymous=False)
	agent = ActorNet(60, 3)

	# load weights
	agent.load_actor(os.path.abspath(__file__).replace('ddpg.py','weights/actor/model.ckpt'))

	map_topic = rospy.get_param('~map_topic','/map')
	scan_topic = rospy.get_param('~scan_topic','/scan')
	path_topic = rospy.get_param('~path_topic','/move_base/TebLocalPlannerROS/local_plan')
	rate = rospy.Rate(rospy.get_param('~rate',10))
#-------------------------------------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(scan_topic, LaserScan, scanCallBack)
	rospy.Subscriber(path_topic, Path, pathCallback)
#-------------------------------------------------------------------------
	pub = rospy.Publisher('ddpg_goal', Marker, queue_size=10) 
	pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
#-------------------------------------------------------------------------
	listener = TransformListener()
		
	q=Point()
	
	points_ddpg=Marker()
	# points_ddpg.header.frame_id= mapData.header.frame_id
	points_ddpg.header.stamp= rospy.Time.now()
	points_ddpg.ns= "markers1"
	points_ddpg.id = 1
	points_ddpg.type = Marker.POINTS
	points_ddpg.action = Marker.ADD
	points_ddpg.pose.orientation.w = 1.0
	points_ddpg.scale.x=0.1
	points_ddpg.scale.y=0.1 
	points_ddpg.color.r = 0.0/255.0
	points_ddpg.color.g = 255.0/255.0
	points_ddpg.color.b = 255.0/255.0
	points_ddpg.color.a=1
	points_ddpg.lifetime = rospy.Duration()
		
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	
	print("start!!!!!!!")
	prev_action = [0.0, 0.0, 0.0]
	while not rospy.is_shutdown():
		if listener.frameExists("/base_link") and listener.frameExists("/map"):
			position, quaternion = listener.lookupTransform("/base_footprint", "/map", listener.getLatestCommonTime("/base_link", "/map"))
			orientation = euler_from_quaternion(quaternion)
			print(position,orientation)
		# print(len(path))
		if len(path)>5: 
			# print("Local Goal(1) :"),
			# print(path[5])
			q.x=path[5][0]
			q.y=path[5][1]
			qq=[]
			qq.append(copy(q))
			points_ddpg.points=qq
			pub.publish(points_ddpg)
			# print("scan :"),
			# print(scandata)
			# print("path :"),
			# print(path)
			'''
			path[5][0]- position
			state = np.array(scandata + prev_action + path[5])
			action = agent.evaluate_actor(state)
			print('Current Decision:',action)
			vel.linear.x = aciton[0]
			vel.linear.y = aciton[1]
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = aciton[2]
			pub2(vel)
			prev_action = action
			'''

#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass
 
 
 
 
