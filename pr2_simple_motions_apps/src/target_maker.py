#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
from geometry_msgs.msg import PointStamped, PoseStamped
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
import tf
from numpy import *

class TargetMaker:
    def __init__(self):
        self.name = rospy.get_name()
        self.point_input = rospy.get_param("~point_input","%s/point_input"%self.name)
        self.pose_input = rospy.get_param("~pose_input","%s/pose_input"%self.name)
        self.output = rospy.get_param("~output","%s/output"%self.name)
        self.move_arm = rospy.get_param("~move_arm",True)
        self.arm = rospy.get_param("~arm","l")
        self.mode = rospy.get_param("~mode","poke")
        self.pub = rospy.Publisher(self.output,GripTarget)
        self.point_sub = rospy.Subscriber(self.point_input,PointStamped,self.handle_point)
        self.pose_sub = rospy.Subscriber(self.pose_input,PoseStamped,self.handle_pose)
        
    def handle_point(self,pt):
        if self.mode == "poke":
            self.output_target(pt=pt,arm=self.arm,roll=pi/2,pitch=pi/2,yaw=0,grip=True)
        elif self.mode == "grab":
            pt.point.z -= 0.07
            self.output_target(pt=pt,arm=self.arm,roll=pi/2,pitch=pi/2,yaw=0,grip=False)
            self.output_target(pt=pt,arm=self.arm,roll=pi/2,pitch=pi/2,yaw=0,grip=True)

    def handle_pose(self,pose):
        pt = PointStamped()
        pt.header.stamp = pose.header.stamp
        pt.header.frame_id = pose.header.frame_id
        pt.point.x = pose.pose.position.x
        pt.point.y = pose.pose.position.y
        pt.point.z = pose.pose.position.z
        (roll,pitch,yaw) = tf.euler_from_quaternion((pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))

        self.output_target(pt=pt,arm=self.arm,roll=roll,pitch=pitch,yaw=yaw,grip=True)
    
    def output_target(self,pt,arm,roll,pitch,yaw,grip):
        target = GripTarget()
        target.point = pt
        target.arm = arm
        target.grip = grip
        target.pitch = pitch
        target.roll = roll
        target.yaw = yaw
        self.pub.publish(target)
        if self.move_arm:
            try:
                move_one_arm = rospy.ServiceProxy("move_one_arm",MoveOneArm)
                resp = move_one_arm(target=target,dur=5.0)
                return resp
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
            
                        


def main(args):
    rospy.init_node("target_maker")
    tm = TargetMaker()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
