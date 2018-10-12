import rospy
from nav_msgs.msg import Path
#from geometry_msgs.msg import Point, Twist
from dualarmbot_bringup.msg import commendMsg
from numpy import atan2

class cmd_publisher:

    def __init__(self):
        self.nodename='cmd_publisher'

    def pathCallback(self,data):
        self.path=[]
        for pose in data.poses:
            path.append(list([pose.pose.position.x,pose.pose.position.y]))

    def node(self):
        path_topic = rospy.get_param('~path_topic','/move_base/DWAPlannerROS/global_plan')
        pub=rospy.Publisher('/ns1/cmd_msg',queue_size=100)
        rospy.Subscriber(path_topic, Path, self.pathCallback)
        rospy.init_node(self.nodename, anonymous=True)
        pose_cmd=commendMsg()
        #pose_cmd.xd
        #pose_cmd.yd
        #pose_cmd.phid

        while not rospy.is_shutdown():
            [pose_cmd.xd,pose_cmd.yd]=self.path[0]
            if len(self.path)>1:
                pose_cmd.phid=atan2(self.path[1][1]-self.path[0][1],self.path[1][0]-self.path[0][0])
            else:
                pass
            pub.publish(pose_cmd)


if __name__=='__main__':
    cmd_pub=cmd_publisher()
    try:
        cmd_pub.node()
    except rospy.ROSInterruptException: pass

