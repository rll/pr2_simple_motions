#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
import actionlib
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *

# Add a location to the location_server
def add_location(location,name):
    try:
        add_location = rospy.ServiceProxy("locations/add_location",AddLocation)
        resp = add_location(name=name,location=location)
        return resp
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)
    
# Take a snapshot of the robot's current setup, w.r.t a given frame, and add it to the location server    
def add_current_location(name,frame):
    try:
        add_current_location = rospy.ServiceProxy("locations/add_current_location",AddCurrentLocation)
        resp = add_current_location(name=name,frame=frame)
        return resp
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)

#Adds a dictionary of locations        
def add_locations(locations_dict):
    for location_name,location in locations_dict.iteritems():
        add_location(location,location_name)
    return True

#Call a location with a given duration    
def call_location(location_name,dur=5.0,params=[]):
    try:
        execute_location = rospy.ServiceProxy("locations/%s"%location_name,ExecuteLocation)
        resp = execute_location(dur,params)
        return resp.success
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%e)

