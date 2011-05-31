#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_base_motions")
import rospy
import sys
from geometry_msgs.msg import PointStamped
from pr2_simple_motions_msgs.msg import *
from pr2_simple_motions_srvs.srv import *
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from numpy import *

#Goes to various stances
class BoxDetector:

    def __init__(self):
        self.listening = False
        self.clear_srv = rospy.Service("box_detector/start_listening",Empty,self.start_listening)
        self.laser_sub = rospy.Subscriber("/base_scan",LaserScan,self.laser_callback)
        
        
    def laser_callback(self,laser_scan):
        if self.listening:
            closest_pt =  min([scan for scan in laser_scan.ranges if scan > 0.01])
            if closest_pt < 0.17:
                self.found_close_point(laser_scan)
            
    def found_close_point(self,laser_scan):
        service_name = "found_box"
        try:
            found_proxy = rospy.ServiceProxy(service_name, Empty)
            found_proxy()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.listening = False
        
        
    def start_listening(self,req):
        self.listening = True
        return []
        
    
def main(args):
    rospy.init_node("primitives")
    box_detector = BoxDetector()
    rospy.spin() 
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
