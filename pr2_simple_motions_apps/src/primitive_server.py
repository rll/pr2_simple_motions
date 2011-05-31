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
import StanceUtils
import yaml
import os.path

"""
Better documentation to come. For ease of use, see PrimitiveUtils.py

The PrimitiveServer is a natural extension of the StanceServer. While any position of the robot can be stored as a stance, we generally want
to string them together to execute a rehearsed motion. To do this, we define a primitive: a sequence of stances to be performed at
specified speeds.


/primitives/add_primitive
---
Adds a primitive to the server
---



/primitives/save_primitives
---
Saves primitives to a yaml file, to be loaded later. By default, these are saved in the pr2_simple_motions_apps/config folder.
In the future, they should probably be saved to a local "~/.primitives" directory instead, so write permissions don't get messed up.
The naming system is: [name]_primitives.yaml. e.g. save_primitives("folding") will generate the file /config/folding_primitives.yaml
---

/primitives/load_primitives
---
Loads all primitives from a yaml file. You provide the name (e.g. "folding") and it will load from /config/[name]_primitives.yaml
This will overwrite any primitives which share the same name, but will not remove anything else. 
---

Finally, for every primitive which has been made, there is a new service generated. If I have a primitive "dance", there will be a service
/primitives/dance
Which takes one argument: a list of parameters. At the moment they do nothing, so an empty list will suffice.
e.g.:
rosservice call /primitives/dance []
Will cause the robot to do the given dance.
"""

class PrimitiveServer:
    def __init__(self):
        self.primitives = {}
        self.listener = tf.TransformListener()
        self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
        self.add_primitive_srv = rospy.Service('primitives/add_primitive', AddPrimitive, self.add_primitive_srv)
        self.save_primitives_service = rospy.Service('primitives/save_primitives', SavePrimitives, self.save_primitives_srv)
        self.load_primitives_service = rospy.Service('primitives/load_primitives', LoadPrimitives, self.load_primitives_srv)
        initial_primitives = rospy.get_param("~initial_primitives","default")
        if initial_primitives != "default":
            self.load_primitives_srv(LoadPrimitivesRequest(initial_primitives))
        rospy.loginfo("Primitive Server is ready!")    
    def add_primitive_srv(self,req):
        primitive_name = req.name
        primitive = req.primitive
        return self.add_primitive(primitive_name,primitive)
        
        
    def add_primitive(self,primitive_name,primitive):
        if not primitive_name in self.primitives.keys():
            rospy.Service("primitives/%s"%primitive_name, ExecutePrimitive, lambda req: self.execute_primitive(self.primitives[primitive_name],req))
        self.primitives[primitive_name] = primitive
        
        return []
        
    
    def execute_primitive(self,primitive,req):
        for weight_point in primitive.weight_points:
            stance_name = weight_point.stance_name
            dur = weight_point.dur
            params = weight_point.params
            resp = StanceUtils.call_stance(stance_name,dur,params)
            #if not resp:
            #    return ExecutePrimitiveResponse(False)
        return ExecutePrimitiveResponse(True)
        

    def save_primitives_srv(self,req):
        filename = "%s_primitives.yaml"%req.label
        filepath = "%s/%s"%(self.config_dir,filename)
        savefile = open(filepath,'w')
        yaml.dump(self.dict_to_yaml(self.primitives),savefile)
        return SavePrimitivesResponse()
        
    def load_primitives_srv(self,req):
        filename = "%s_primitives.yaml"%req.label
        filepath = "%s/%s"%(self.config_dir,filename)
        savefile = open(filepath)
        new_primitives = self.yaml_to_dict(yaml.load(savefile))
        for k,v in new_primitives.items():
            self.add_primitive(k,v)
        return LoadPrimitivesResponse()

    def dict_to_yaml(self,primitive_dict):
        newdict = {}
        for name,primitive in primitive_dict.items():
            weight_points = []
            for wp in primitive.weight_points:
                weight_points.append({'stance_name':wp.stance_name,'dur':wp.dur,'params':list(wp.params)})
            newdict[name] = {'weight_points':weight_points}
        return newdict
    
    def yaml_to_dict(self,primitive_yaml):
        newdict = {}
        for key,value in primitive_yaml.items():
            wps = [WeightPoint(*d.values()) for d in value['weight_points']]
            newdict[key] = Primitive(weight_points=wps)
        return newdict
        
def main(args):
    rospy.init_node("primitive_server")
    ps = PrimitiveServer()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
