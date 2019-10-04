#!/usr/bin/env python

#NOTE TO INSTRUCTORS: This code largely written by Derek Thompson of team BRZ. He provded it to me as a template
#I made some small changes 


import rospy
import signal
import sys
import math
import numpy as np
import time
from std_msgs.msg import Int32, String, Float32MultiArray, Bool, Float32, Empty
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf import transformations as tfs
import tf


from visualization_msgs.msg import Marker, MarkerArray

global_point_data=Point()

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)


class bebop_display:
    def __init__(self):        

        self.vichile_pub = rospy.Publisher("/auto/rviz/vehicle", MarkerArray, queue_size=1)
        self.object_pub = rospy.Publisher("/auto/rviz/object", MarkerArray, queue_size=1)
        self.spline_pub = rospy.Publisher("/auto/rviz/spline", MarkerArray, queue_size=1)

        # static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
        self.tbr = tf.TransformBroadcaster()



        rospy.Subscriber("/bebop/odom", Odometry, self.callback, "odom")
        #rospy.Subscriber("/auto/auto_drive", Auto_Driving_Msg, self.callback, "cmds")
        #rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged,self.callback, "battery")
        #rospy.Subscriber("/bebop/states/common/CommonState/WifiSignalChanged", CommonCommonStateWifiSignalChanged,self.callback, "wifi")
        rospy.Subscriber("/bebop/waypoint_ilya", Point, self.callback,'point')


        # Variables
        self.battery = -1
        self.rssi = 0

        self.create_markers()




    def create_markers(self):

        self.vehicle_marker = MarkerArray()
        point_marker = Marker()
        point_marker.header.frame_id = "vehicle_frame"          #frame that marker will be plotted in
        point_marker.header.stamp    = rospy.get_rostime()      
        point_marker.ns = "vehicle"                             #vehicle name
        point_marker.id = 0                                     # marker id in namespace
        point_marker.type = 1                                   # prism                           
        point_marker.action = 0                                 # should always be zero
        point_marker.scale.x = .3                           
        point_marker.scale.y = .12
        point_marker.scale.z = .08
        point_marker.pose.orientation.w = 1                     # should always be 1
        point_marker.pose.position.x = 0                        # position in the vehicle frame
        point_marker.pose.position.y = 0                        # position in the vehicle frame
        point_marker.pose.position.z = 0                        # position in the vehicle frame
        point_marker.color.r = 0                        
        point_marker.color.g = 0
        point_marker.color.b = 1
        point_marker.color.a = 1.0                              # Opacity, should always be 1
        point_marker.lifetime = rospy.Duration(0)   
        self.vehicle_marker.markers.append(point_marker)        # append whatever markers you need


        self.point_marker = MarkerArray()
        point_marker = Marker()
        point_marker.header.frame_id = "point_frame"
        point_marker.header.stamp    = rospy.get_rostime()
        point_marker.ns = "point"
        point_marker.id = 0
        point_marker.type = 0
        point_marker.action = 0
        point_marker.scale.x = .1
        point_marker.scale.y = .1
        point_marker.scale.z = .2
        point_marker.pose.orientation.w = 1
        point_marker.color.r = 1
        point_marker.color.a = 1.0
        point_marker.lifetime = rospy.Duration(0)
        self.point_marker.markers.append(point_marker)
        



    def callback(self,data,args):
        global global_point_data
        if args == 'odom':
            #print('working')
            #print(global_point_data)
            quat = data.pose.pose.orientation
            pos = data.pose.pose.position
            

            # publishers for frame transforms 
            # defines differences between frames
            # difference to vehicle frame from odom frame
            self.tbr.sendTransform((pos.x,pos.y,pos.z),(quat.x,quat.y,quat.z,quat.w),rospy.get_rostime(),'vehicle_frame', "odom")
            self.vichile_pub.publish(self.vehicle_marker)


        if args == 'point':
            global_point_data=data
            #quat = data.pose.pose.orientation
            #print('point data recieved')
            pos = data
            
            self.tbr.sendTransform((pos.x,pos.y,pos.z),(0,0,0,1),rospy.get_rostime(),'point_frame', "odom")
            self.vichile_pub.publish(self.point_marker)



        elif args == 'cmds':
            pass



        elif args == 'battery':
            self.battery = data.data

        elif args == 'wifi':
            self.rssi = data.data
            print 'Batt: ',self.battery,', RSSI: ',self.rssi






if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    # initialize node
    rospy.init_node('Rviz_display', anonymous=False)

    
    rviz_displayer = bebop_display()
    

    rospy.spin()
