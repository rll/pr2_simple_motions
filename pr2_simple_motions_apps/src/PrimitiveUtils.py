#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
import actionlib
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
from numpy import *

#Takes a set of stances, durations, and parameters, and creates a primitive
def make_primitive(stances,durs,params):
    assert len(stances) == len(durs) == len(params)
    new_primitive = Primitive()
    for i in range(len(stances)):
        weight_point = WeightPoint(stance_name=stances[i],dur=durs[i],params=params[i])
        new_primitive.weight_points.append(weight_point)
    return new_primitive

#Adds a primitive to the primitive server
def add_primitive(primitive,name):
    try:
        add_primitive = rospy.ServiceProxy("primitives/add_primitive",AddPrimitive)
        resp = add_primitive(name=name,primitive=primitive)
        return resp
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
    
#Adds a dictionary of primitives to the server        
def add_primitives(primitives_dict):
    for primitive_name,primitive in primitives_dict.iteritems():
        add_primitive(primitive,primitive_name)
    return True

#Wrapper for calling a primitive    
def call_primitive(primitive_name,params=[]):
    try:
        execute_primitive = rospy.ServiceProxy("primitives/%s"%primitive_name,ExecutePrimitive)
        resp = execute_primitive(params)
        return resp.success
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
        return False
