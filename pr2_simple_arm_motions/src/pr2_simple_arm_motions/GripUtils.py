#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_arm_motions")
import rospy
import sys
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped, QuaternionStamped
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
from numpy import pi, cos, sin, abs
import tf
import time
import thread
import copy

#Hard voded variables for how to modify grip if you've failed to reach a grasp point
APPROACH_UP = 0.01
APPROACH_BACK = 0.005
SCOOT_AMT = 0.015
#INIT_SCOOT_AMT = 0.03

global_listener = None

#def go_to_pt(point,roll,pitch,yaw,grip,arm,dur=5.0,x_offset=0.0,y_offset=0.0,z_offset=0.0,link_frame=""):
#Forces you to go to a point, by whatever means necessary
def force_go_to_pt(point,roll,pitch,yaw,grip,arm,dur=5.0,x_offset=0.0,y_offset=0.0,z_offset=0.0,link_frame=''):
    #print "Trying regular"
    #if go_to_pt(point,roll,pitch,yaw,grip,arm,dur,x_offset,y_offset,z_offset,link_frame):
    #    return True
    for new_x_offset in (x_offset,x_offset - 0.01, x_offset + 0.01,x_offset-0.02,x_offset+0.02):
        for new_y_offset in (y_offset,y_offset - 0.01, y_offset + 0.01,y_offset - 0.02,y_offset+0.02):
            for new_yaw in (yaw, yaw-pi/4, yaw+pi/4):
                for new_pitch in [pitch]:#, pi/4,pi/2,3*pi/4):
                    if go_to_pt(point,roll,new_pitch,new_yaw,grip,arm,dur,new_x_offset,new_y_offset,z_offset,link_frame):
                        return [new_pitch,new_yaw,new_x_offset,new_y_offset]
    return False
    

#Grab a 3D point with the gripper, iteratively increasing your approach til you've picked something up
def grab_point(point,roll=pi/2,pitch=pi/4,yaw=0,arm='l',x_offset=0.0,y_offset=0.0,z_offset=0.0,link_frame='',approach=True,INIT_SCOOT_AMT=0.03):
    x = point.point.x + x_offset
    y = point.point.y + y_offset
    z = point.point.z + z_offset
    frame = point.header.frame_id
    return grab(x=x,y=y,z=z,roll=roll,pitch=pitch,yaw=yaw,arm=arm,frame=frame,link_frame=link_frame,approach=approach,INIT_SCOOT_AMT=INIT_SCOOT_AMT)

#Grab a point, iteratively increasing the approach til you've picked something up.
def grab(x,y,z,roll=pi/2,pitch=pi/4,yaw=0,frame="torso_lift_link",arm='l',link_frame='',approach=True,INIT_SCOOT_AMT=0.03):
    if approach:
        dx = -1*APPROACH_BACK * cos(yaw)
        dy = -1*APPROACH_BACK * sin(yaw)
        success = go_to(x=x+dx,y=y+dy,z=z+APPROACH_UP,roll=roll,pitch=pitch,yaw=yaw,grip=False,frame=frame,arm=arm,dur=5.0,link_frame=link_frame) 
        if not success:
            return False
        new_x = x+INIT_SCOOT_AMT*cos(yaw)
        new_y = y+INIT_SCOOT_AMT*sin(yaw)
    else:
        new_x = x
        new_y = y
    has_obj = False
    i = 0
    num_tries = 4
    if pitch==pi/2:
        num_tries = 5
    while not has_obj:
        
        #success = go_to(x=new_x,y=new_y,z=z+0.01*(1-sin(pitch)),roll=roll,pitch=pitch,yaw=yaw,grip=False,frame=frame,arm=arm,dur=3.0,link_frame=link_frame)
        success = go_to(x=new_x,y=new_y,z=z+0.01*(1-sin(pitch))-0.005,roll=roll,pitch=pitch,yaw=yaw,grip=False,frame=frame,arm=arm,dur=3.0,link_frame=link_frame)
        #if not success:
        #    return False
        print "Closing gripper"
        close_gripper(arm)
        has_obj = has_object(arm)
        print "has_object(%s) returned %s"%(arm,has_obj)
        if not has_obj:
            print "Opening gripper"
            open_gripper(arm)
            if pitch==pi/2:
                success = go_to(x=new_x,y=new_y,z=z+0.03,roll=roll,pitch=pitch,yaw=yaw,grip=False,frame=frame,arm=arm,dur=3.0,link_frame=link_frame)
            new_x += SCOOT_AMT * cos(yaw)
            new_y += SCOOT_AMT * sin(yaw)
            i += 1
            if i == num_tries:
                new_x = x - INIT_SCOOT_AMT
                new_y = y
            elif i == 2*num_tries:
                new_x = x + INIT_SCOOT_AMT
                new_y = y 
            if pitch==pi/2:
                go_to(x=new_x,y=new_y,z=z+0.03,roll=roll,pitch=pitch,yaw=yaw,grip=False,frame=frame,arm=arm,dur=3.0,link_frame=link_frame)
                
        rospy.sleep(0.5)
    return True

#Move the gripper to a given x,y,z point        
def go_to(x,y,z,roll,pitch,yaw,grip,frame,arm,dur=5.0,link_frame=""):
    pt = PointStamped()
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = frame
    pt.point.x = x
    pt.point.y = y
    pt.point.z = z
    return go_to_pt(point=pt,roll=roll,pitch=pitch,yaw=yaw,grip=grip,arm=arm,dur=dur,link_frame=link_frame)

#Move both grippers simultaneously    
def go_to_multi (x_l,y_l,z_l,roll_l,pitch_l,yaw_l,grip_l,frame_l
                ,x_r,y_r,z_r,roll_r,pitch_r,yaw_r,grip_r,frame_r
                ,link_frame_l = "", link_frame_r = ""
                ,dur=5.0):
    pt_l = PointStamped()
    pt_l.header.stamp = rospy.Time.now()
    pt_l.header.frame_id = frame_l
    pt_l.point.x = x_l
    pt_l.point.y = y_l
    pt_l.point.z = z_l
    pt_r = PointStamped()
    pt_r.header.stamp = rospy.Time.now()
    pt_r.header.frame_id = frame_r
    pt_r.point.x = x_r
    pt_r.point.y = y_r
    pt_r.point.z = z_r
    return go_to_pts    (point_l=pt_l,roll_l=roll_l,pitch_l=pitch_l,yaw_l=yaw_l,grip_l=grip_l
                        ,point_r=pt_r,roll_r=roll_r,pitch_r=pitch_r,yaw_r=yaw_r,grip_r=grip_r
                        ,link_frame_l=link_frame_l, link_frame_r=link_frame_r
                        ,dur=dur)
                
#Move the gripper to a StampedPoint. Relies on the "move_one_arm" service
#FIXME: should remap service name instead of hardcoding 
def go_to_pt(point,roll,pitch,yaw,grip,arm,dur=5.0,x_offset=0.0,y_offset=0.0,z_offset=0.0,link_frame="",verbose=False):
    new_point = PointStamped()
    new_point.header.stamp = point.header.stamp
    new_point.header.frame_id = point.header.frame_id
    new_point.point.x = point.point.x + x_offset
    new_point.point.y = point.point.y + y_offset
    new_point.point.z = point.point.z + z_offset
    target = GripTarget(point=new_point,arm=arm,grip=grip,pitch=pitch,roll=roll,yaw=yaw,link_frame=link_frame)
    try:
        move_one_arm = rospy.ServiceProxy("move_one_arm",MoveOneArm)
        if verbose:
            print "Calling move_one_arm at orientation (%f,%f,%f) in frame %s"%(roll,pitch,yaw,point.header.frame_id)
        resp = move_one_arm(target=target,dur=dur)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
        return False
    if verbose:
        print resp.success
        print resp
    return resp.success

#Move the grippers to two StampedPoints. Relies on the "move_both_arms" service.
#FIXME: should remap service name instead of hardcoding 
def go_to_pts(point_l,roll_l,pitch_l,yaw_l,grip_l,point_r,roll_r,pitch_r,yaw_r,grip_r,dur=5.0,link_frame_l="",link_frame_r="",x_offset_l=0.0,y_offset_l=0.0,z_offset_l=0.0,x_offset_r=0.0,y_offset_r=0.0,z_offset_r=0.0,verbose=False):
    point_l = copy.deepcopy(point_l)
    point_r = copy.deepcopy(point_r)
    point_l.point.x += x_offset_l
    point_l.point.y += y_offset_l
    point_l.point.z += z_offset_l
    point_r.point.x += x_offset_r
    point_r.point.y += y_offset_r
    point_r.point.z += z_offset_r
    target_l = GripTarget(point=point_l,arm="l",grip=grip_l,pitch=pitch_l,roll=roll_l,yaw=yaw_l,link_frame=link_frame_l)
    target_r = GripTarget(point=point_r,arm="r",grip=grip_r,pitch=pitch_r,roll=roll_r,yaw=yaw_r,link_frame=link_frame_r)
    try:
        move_both_arms = rospy.ServiceProxy("move_both_arms",MoveBothArms)
        if verbose:
            print "Calling move_both_arms"
        resp = move_both_arms(target_left=target_l,target_right=target_r,dur=dur)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
        return False
    return resp.success

def get_relative_point(x_offset, y_offset, z_offset, arm, frame,listener=None):
    if not listener:
        global global_listener
        if not global_listener:
            global_listener = tf.TransformListener()
        listener = global_listener
    current = PoseStamped()
    
    sourceFrame = "/%s_tip_frame"%arm
    print "sourceFrame: ", sourceFrame
    current.header.frame_id = sourceFrame

    while True:
        try:
            now = rospy.Time.now()
            current.header.stamp = now
            listener.waitForTransform(frame, sourceFrame, rospy.Time.now(), rospy.Duration(1.0))
            transformed = listener.transformPose(frame, current)
            break
        except tf.Exception,e:
            pass

    print "MADE IT THIS FAR"
    
    transformed.pose.position.x += x_offset
    transformed.pose.position.y += y_offset
    transformed.pose.position.z += z_offset
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w])
    point = PointStamped()
    point.header.frame_id = "base_footprint"
    point.header.stamp = now
    point.point.x = transformed.pose.position.x
    point.point.y = transformed.pose.position.y
    point.point.z = transformed.pose.position.z
    return (point,roll,pitch,yaw)    

def go_to_relative(x_offset, y_offset, z_offset, grip, arm, frame,listener=None):
    (point, roll, pitch, yaw) = get_relative_point(x_offset, y_offset, z_offset, arm, frame, listener)
    return go_to_pt(point, roll, pitch, yaw, grip, arm, 5, x_offset, y_offset, z_offset)

def go_to_relative_multi(x_offset_l, y_offset_l, z_offset_l, grip_l, x_offset_r, y_offset_r, z_offset_r, grip_r, frame,listener=None):
    (point_l, roll_l, pitch_l, yaw_l) = get_relative_point(x_offset_l, y_offset_l, z_offset_l, "l", frame, listener)
    (point_r, roll_r, pitch_r, yaw_r) = get_relative_point(x_offset_r, y_offset_r, z_offset_r, "r", frame, listener)
    return go_to_pts(point_l, roll_l, pitch_l, yaw_l, grip_l, point_r, roll_r, pitch_r, yaw_r, grip_r)

#Checks if the gripper is holding an object. Uses the /has_object service, spawned by grip_watcher.py
#FIXME: should remap service name instead of hardcoding    
def has_object(arm):
    resp = False
    try:
        get_joint = rospy.ServiceProxy("return_joint_state",ReturnJointState)
        resp = get_joint("%s_gripper_joint"%arm)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
        rospy.sleep(0.2)
        return self.has_object(arm)
    if resp:
        if arm == 'l':
            return -0.0005 < resp.position < 0.01
        else:
            return -0.0005 < resp.position < 0.01

#Closes the gripper by calling the "close_grippers" service
#FIXME: should remap service name instead of hardcoding 
def close_gripper(arm,extra_tight=False):
    try:
        close_grippers = rospy.ServiceProxy("close_grippers",CloseGrippers)
        resp = close_grippers(arms=arm,extra_tight=extra_tight)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
        return False
    return resp.success

#Close both grippers    
def close_grippers(extra_tight=False):
    return close_gripper("b")

#Opens the gripper by calling the "open_grippers" service
#FIXME: should remap service name instead of hardcoding     
def open_gripper(arm,extra_tight=False):
    try:
        close_grippers = rospy.ServiceProxy("open_grippers",OpenGrippers)
        resp = close_grippers(arms=arm)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
        return False
    return resp.success
    
#Opens both grippers   
def open_grippers(extra_tight=False):
    return open_gripper("b")

#Recalls an arm
def recall_arm(arm,grip=False):
    height = 0.35
    lateral_amount = 0.65
    forward_amount = 0.3

    if arm == 'b':
        lateral_amount_r = lateral_amount * -1
        lateral_amount_l = lateral_amount

        return go_to_multi(     x_l=forward_amount,         y_l=lateral_amount_l,   z_l=height,
                                roll_l=0,                   pitch_l=0,              yaw_l=0, 
                                frame_l="torso_lift_link",
                                grip_l=grip,
                                x_r=forward_amount,         y_r=lateral_amount_r,   z_r=height,
                                roll_r=0,                   pitch_r=0,              yaw_r=0,
                                frame_r="torso_lift_link",
                                grip_r=grip,           dur=2.5)
    else:
        if arm == 'r':
            lateral_amount = lateral_amount * -1
        return go_to(   x=forward_amount, y=lateral_amount, z=height,
                        roll=0, pitch=0, yaw=0, grip=grip,
                        frame="torso_lift_link", dur=2.5, arm=arm)

def arms_up(grip=False):
    return recall_arm('b', grip)

def shake_arm(arm, num_shakes):

    if num_shakes < 1:
        rospy.logwarn("Must shake at least once.")
        return False

    forward_amount  = 0.375
    height          = 0.65
    drop_amount     = 0.25
    lateral_amount  = 0.05

    if arm=="r":
        yaw_multiplier = 1
        y_multiplier = -1
    else:
        yaw_multiplier = -1
        y_multiplier = 1
        
    for i in range(num_shakes):

        if i == 0:
            duration = 4.0

        if not go_to(x=forward_amount, y=0, z=height,
                     roll=0, pitch=0, yaw=yaw_multiplier*pi/2, grip=True,
                     frame="table_height",arm=arm,dur=duration):
            return False 

        duration = .5

        if not go_to(x=forward_amount, y=lateral_amount*y_multiplier, z=height-drop_amount,
                     roll=0, pitch=pi/4, yaw=yaw_multiplier*pi/2, grip=True,
                     frame="table_height", arm=arm,dur=duration):
            return False

    if not go_to(x=forward_amount,y=0,z=height,
        roll=0,pitch=0,yaw=yaw_multiplier*pi/2,grip=True,
            frame="table_height",arm=arm,dur=duration):
        return False 

    return True 

def shake_arms(num_shakes, width):
    if num_shakes < 1:
        rospy.logwarn("Must shake at least once.")
        return False

    forward_amount  = 0.45
    height          = 0.625
    drop_amount     = 0.35
    lateral_amount  = 0.1

    for i in range(num_shakes):

        if i == 0:
            duration = 4.0


        if not go_to_multi(x_l=forward_amount+.2,y_l=width*.95/2.0,z_l=height,
                       roll_l=0,pitch_l=0,yaw_l=-pi/4,grip_l=True,
                       x_r=forward_amount+.2,y_r=-width*.95/2.0,z_r=height,
                       roll_r=0,pitch_r=0,yaw_r=pi/4,grip_r=True,
                       frame_l="table_height",frame_r="table_height",dur=duration):
            return False
        duration = 0.6
        if not go_to_multi(x_l=forward_amount,y_l=width/2.0,z_l=height,
                           roll_l=0,pitch_l=0,yaw_l=-pi/2,grip_l=True,
                           x_r=forward_amount,y_r=-width/2.0,z_r=height,
                           roll_r=0,pitch_r=0,yaw_r=pi/2,grip_r=True,
                           frame_l="table_height",frame_r="table_height",dur=duration):
            return False

        if width < 0.30:
            rospy.logwarn("Cannot shake at less than 0.30m apart; tried to do %f" % width)
            break

        duration = 0.6
        if not go_to_multi(x_l=forward_amount,y_l=width/2.0-lateral_amount,z_l=height-drop_amount,
                           roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                           x_r=forward_amount,y_r=-width/2.0+lateral_amount,z_r=height-drop_amount,
                           roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                           frame_l="table_height",frame_r="table_height",dur=duration):
            return False

    width = width * .90

    if width > .775:
        width = .775

    if not go_to_multi(x_l=forward_amount+.2,y_l=width/2.0,z_l=height,
                       roll_l=0,pitch_l=0,yaw_l=-pi/4,grip_l=True,
                       x_r=forward_amount+.2,y_r=-width/2.0,z_r=height,
                       roll_r=0,pitch_r=0,yaw_r=pi/4,grip_r=True,
                       frame_l="table_height",frame_r="table_height",dur=duration):
        return False

    return True


def get_joint_position(joint_name):
    resp = False
    try:
        joint_state_srv = rospy.ServiceProxy("return_joint_state",ReturnJointState)
        resp = joint_state_srv(joint_name)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
    return resp.position

def current_grip(arm):
    return get_joint_position("%s_gripper_r_finger_joint"%arm) < 0.2

def angle_more(arm,roll=0,pitch=0,yaw=0,dur=1.0):
    go_to(  x=0,y=0,z=0,
            roll=roll,pitch=pitch,yaw=yaw,
            grip=current_grip(arm),
            frame="%s_gripper_tool_frame"%arm,arm=arm,dur=dur,
            link_frame="%s_gripper_tool_frame"%arm)

class RollWatcher:

    def __init__(self,arm):
        self.arm = arm
        self.clear()

    def clear(self):
        self.start_position = None
        self.end_position = None

    def start_watching(self):
        self.start_position = self.get_position()

    def stop_watching(self):
        self.end_position = self.get_position()

    def adjust(self):
            roll_difference = self.end_position - self.start_position
            roll_direction = -1 if roll_difference > 0 else 1
            is_gripping = current_grip(self.arm)
            while abs(roll_difference) >= pi:
                for iter in range(3):
                    go_to(  x=0,y=0,z=0,
                            roll=roll_direction*2*pi/3,pitch=0,yaw=0,
                            grip=is_gripping,
                            frame="%s_gripper_tool_frame"%self.arm,arm=self.arm,dur=1.0,
                            link_frame="%s_gripper_tool_frame"%self.arm)
                roll_difference += roll_direction*2*pi
            self.clear()

    def get_position(self):
        return get_joint_position("%s_wrist_roll_joint"%self.arm) + get_joint_position("%s_forearm_roll_joint"%self.arm)

