#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_utils")
import rospy
import sys
from numpy import *
import tf
import time
from geometry_msgs.msg import PoseStamped

def capture_pose(frame):
    listener = tf.TransformListener()
    rospy.sleep(1.0)
    l_origin = origin("l_tip_frame")
    listener.waitForTransform(frame,"l_tip_frame",rospy.Time.now(),rospy.Duration(10.0))
    left_pose = listener.transformPose(frame,l_origin)
    r_origin = origin("r_tip_frame")
    listener.waitForTransform(frame,"r_tip_frame",rospy.Time.now(),rospy.Duration(10.0))
    right_pose = listener.transformPose(frame,r_origin)
    print "Base_Frame: %s"%frame
    print_pose(left_pose,"Left Gripper")
    print_pose(right_pose,"Right Gripper")
    now = rospy.Time.now()
    listener.waitForTransform("odom_combined","base_footprint",now,rospy.Duration(10.0))
    (trans,rot) = listener.lookupTransform("odom_combined","base_footprint",now)
    (x,y,z) = trans
    (ox,oy,oz,ow) = rot
    print "Translation from Base to Odom: %f,%f,%f.\n"%(x,y,z)
    print "Rotation from Base to Odom: (%f,%f,%f)"%(tf.transformations.euler_from_quaternion(rot))
    
def print_pose(pose,name):
    print "%s:"%name
    print "\tx: %f , y: %f , z: %f"%(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z)
    print "\troll: %f ,pitch: %f ,yaw: %f"%quat_to_euler(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)
    print ""

def quat_to_euler(x,y,z,w):
        return tf.transformations.euler_from_quaternion((x,y,z,w))

def origin(frame):
    pt = PoseStamped()
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = frame
    return pt

def main(args):
    rospy.init_node("pose_capturer")
    if len(args) == 1:
        frame = args[0]
    else:
        print "Usage: pose_capturer [frame]"
        exit(1)
    capture_pose(frame)
   # rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
