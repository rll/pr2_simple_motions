#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
import actionlib
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from numpy import *
import tf
import time
import thread
import yaml
import os.path
from pr2_simple_arm_motions import GripUtils
import re

"""
Better documentation to come. For ease of use, see LocationUtils.py

The LocationServer is a means of recording robot locations in a frame, to revisit later (rather than putting actual
numbers in the code). It provides a few services:


/locations/add_location
---
Adds a location to the server.
---

/locations/add_current_location
---
Takes a snapshot of the robot's base location w.r.t. a frame, and adds it to the server.
---

/locations/save_locations
---
Saves locations to a yaml file, to be loaded later. By default, these are saved in the pr2_simple_motions_apps/config folder.
In the future, they should probably be saved to a local "~/.locations" directory instead, so write permissions don't get messed up.
The naming system is: [name]_locations.yaml. e.g. save_locations("folding") will generate the file /config/folding_locations.yaml
---

/locations/load_locations
---
Loads all locations from a yaml file. You provide the name (e.g. "folding") and it will load from /config/[name]_locations.yaml
This will overwrite any locations which share the same name, but will not remove anything else. 
---

Finally, for every location which has been made, there is a new service generated. If I have a location "pickup", there will be a service
/locations/pickup
Which takes two arguments: a duration (e.g. 5.0) and a list of parameters. At the moment you can't define parametrized locations, so an empty list will suffice.
e.g.:
rosservice call /locations/pickup 5.0 []
Will cause the robot to move to the pickup locaiton at a duration of 5 seconds.

NOTE: requires the map frame to be in tf.
"""

class LocationServer:
    def __init__(self):
        self.locations = {}
        self.listener = tf.TransformListener()
        self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
        self.add_location_service = rospy.Service('locations/add_location', AddLocation, self.add_location_srv)
        self.add_current_location_service = rospy.Service('locations/add_current_location', AddCurrentLocation, self.add_current_location_srv)
        self.save_locations_service = rospy.Service('locations/save_locations', SaveLocations, self.save_locations_srv)
        self.load_locations_service = rospy.Service('locations/load_locations', LoadLocations, self.load_locations_srv)
        #initial_locations = rospy.get_param("~initial_locations","default")
        #if initial_locations != "default":
        #    self.load_locations_srv(LoadStancesRequest(initial_locations))
        rospy.loginfo("Location Server is ready!")
            
    def pause(self,req):
        rospy.sleep(req.dur)
        return ExecuteStanceResponse(True)
        
    def add_location_srv(self,req):
        location_name = req.name
        location = req.location
        return self.add_location(location_name,location)
        
        
    def add_current_location_srv(self,req):
        location_name = req.name
        frame = req.frame
        location = self.current_pose(frame)
        return self.add_location(location_name, location)
        
    def add_location_srv(self,req):
        location_name = req.name
        location = req.target
        return self.add_location(location_name, location)

    def add_location(self,location_name,location):
        print "Adding location '%s':\n%s"%(location_name,location)
        my_location = location
        #If the location has not been added before, we make a server
        if location_name not in self.locations.keys():
              rospy.Service("locations/%s"%location_name, ExecuteLocation, lambda req: self.execute_location(self.locations[location_name],req))
        self.locations[location_name]=my_location
        return []
        
    def current_pose(self,frame):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_footprint"
        self.listener.waitForTransform(frame,pose.header.frame_id,pose.header.stamp,rospy.Duration(20.0))
        new_pose = self.listener.transformPose(frame,pose)
        return new_pose
    
    def execute_location(self,location,req):
        dur = req.dur
        print "Preparing to send goal..."
        srv = rospy.ServiceProxy("set_pose", MoveBaseToPose)
        resp = srv(location)
        print "DONE!"
        return ExecuteLocationResponse(True)
        
    def copy_location(self,location):
        new_location = Stance()
        new_location.arms = location.arms
        new_location.x_left = location.x_left
        new_location.y_left = location.y_left
        new_location.z_left = location.z_left
        new_location.roll_left = location.roll_left
        new_location.pitch_left = location.pitch_left
        new_location.yaw_left = location.yaw_left
        new_location.grip_left = location.grip_left
        new_location.frame_left = location.frame_left
        new_location.x_right = location.x_right
        new_location.y_right = location.y_right
        new_location.z_right = location.z_right
        new_location.roll_right = location.roll_right
        new_location.pitch_right = location.pitch_right
        new_location.yaw_right = location.yaw_right
        new_location.grip_right = location.grip_right
        new_location.frame_right = location.frame_right
        return new_location
        
    def get_new_locations(self):
        locations = {}
        for k,v in self.locations.items():
            #if not k in self.default_locations:
                locations[k] = v
        return locations

    def save_locations_srv(self,req):
        filename = "%s_locations.yaml"%req.label
        filepath = "%s/%s"%(self.config_dir,filename)
        savefile = open(filepath,'w')
        yaml.dump(self.dict_to_yaml(self.get_new_locations()),savefile)
        return SaveLocationsResponse()
        
    def load_locations_srv(self,req):
        filename = "%s_locations.yaml"%req.label
        filepath = "%s/%s"%(self.config_dir,filename)
        savefile = open(filepath)
        new_locations = self.yaml_to_dict(yaml.load(savefile))
        for k,v in new_locations.items():
            self.add_location(k,v)
        return LoadLocationsResponse()

    def dict_to_yaml(self,location_dict):
        newdict = {}
        for name,location in location_dict.items():
            newdict[name] = {
                            'header.stamp':location.header.stamp, 'header.frame_id':location.header.frame_id,
                            'pose.position.x':location.pose.position.x, 'pose.position.y':location.pose.position.y, 'pose.position.z':location.pose.position.z,
                            'pose.orientation.x':location.pose.orientation.x, 'pose.orientation.y':location.pose.orientation.y,
                            'pose.orientation.z':location.pose.orientation.z, 'pose.orientation.w':location.pose.orientation.w
                            }
        return newdict
    
    def yaml_to_dict(self,location_yaml):
        newdict = {}
        for key,value in location_yaml.items():
            
            new_location = PoseStamped()
            for k,v in value.items():
                object_to_change = new_location            
                while(True):                
                    dotpattern = re.match("(.+?)\.(.+)",k)
                    if not dotpattern:
                        break
                    else:
                        object_to_change = getattr(object_to_change,dotpattern.group(1))
                        k = dotpattern.group(2)                      
                print "Setting new_location.%s to %s"%(k,v)
                setattr(object_to_change,k,v)
            newdict[key] = new_location
        return newdict
        
def main(args):
    rospy.init_node("location_server")
    ls = LocationServer()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
