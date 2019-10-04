#!/usr/bin/env python


import rospy
import signal
import sys
import math
import numpy as np
import time
import matplotlib
from std_msgs.msg import Int32, String, Float32MultiArray, Bool, Float32, Empty
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf import transformations as tfs
import tf


from visualization_msgs.msg import Marker, MarkerArray

global_waypoint=np.array([0.,0.,0.])
global_odompos=np.array([0.,0.,0.,])

global_file = open('Plotting_output.txt','a') #append only file


def signal_handler(_, __):
    # enable Ctrl+C of the script
    print('Closing file')
    global_file.close()
    print('Closing script')
    sys.exit(0)

def datalistener():
    #okay so we start this listener node

    rospy.init_node('datalistener', anonymous=True, log_level=rospy.WARN)
    rospy.Subscriber("/bebop/odom", Odometry, logOdom, "odom")
    rospy.Subscriber("/bebop/waypoint_ilya", Point, logPoint,'point')

    rospy.spin()

    


def logOdom(msg,args):
    global global_odompos
    pos = msg.pose.pose.position
    global_odompos= np.array([pos.x,pos.y,pos.z])

    print(global_odompos)
    #print('The dimensions of x_gyro_rad are: ' + str(x_gyro_rad.shape))
    global_file.write( 'ODOM, ' + str(pos.x) + ',' + str(pos.y) + ',' + str(pos.z))
    global_file.write('\n')

def logPoint(msg,args):
    global global_waypoint
    pos = msg
    global_waypoint= np.array([pos.x,pos.y,pos.z])
    global_file.write( 'WAYPOINT, ' + str(pos.x) + ',' + str(pos.y) + ',' + str(pos.z))
    global_file.write('\n')


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    datalistener()
