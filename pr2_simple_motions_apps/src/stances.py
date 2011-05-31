#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
import actionlib
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
from numpy import *

def add_stance(stance,name):
    try:
        add_stance = rospy.ServiceProxy("stances/add_stance",AddStance)
        resp = add_stance(name=name,stance=stance)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
    return resp
    
def add_current_stance(name,frame,arms):
    try:
        add_current_stance = rospy.ServiceProxy("stances/add_current_stance",AddCurrentStance)
        resp = add_current_stance(name=name,frame=frame,arms=arms)
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
        
def add_stances(stances_dict):
    for stance_name,stance in stances_dict.iteritems():
        add_stance(stance,stance_name)
    return True
