#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
import actionlib
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
from numpy import *

# Add a stance to the stance_server
def add_stance(stance,name):
    try:
        add_stance = rospy.ServiceProxy("stances/add_stance",AddStance)
        resp = add_stance(name=name,stance=stance)
        return resp
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
    
# Take a snapshot of the robot's current setup, w.r.t a given frame, and add it to the stance server    
def add_current_stance(name,frame,arms):
    try:
        add_current_stance = rospy.ServiceProxy("stances/add_current_stance",AddCurrentStance)
        resp = add_current_stance(name=name,frame=frame,arms=arms)
        return resp
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)

#Adds a dictionary of stances        
def add_stances(stances_dict):
    for stance_name,stance in stances_dict.iteritems():
        add_stance(stance,stance_name)
    return True

#Call a stance with a given duration    
def call_stance(stance_name,dur,params=[]):
    try:
        execute_stance = rospy.ServiceProxy("stances/%s"%stance_name,ExecuteStance)
        resp = execute_stance(dur,params)
        return resp.success
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)

#Wrappers for a few common stances:

def pause(dur):
    call_stance("pause",dur,[])
  
def open_left():
    call_stance("open_left",3.0)
    
def open_right():
    call_stance("open_right",3.0)
    
def open_both():
    call_stance("open_both",3.0)
    
def close_left():
    call_stance("close_left",3.0)
    
def close_right():
    call_stance("close_right",3.0)
    
def close_both():
    call_stance("close_both",3.0)
    
def close_gripper(arm):
    if arm == "b":
        close_both()
    elif arm == "l":
        close_left()
    elif arm == "r":
        close_right()

def open_gripper(arm):
    if arm == "b":
        open_both()
    elif arm == "l":
        open_left()
    elif arm == "r":
        open_right()
