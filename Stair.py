#!/usr/bin/env python


## Helical Trajectory Commander that listens to Bebop odometry messages
## And commands bebop pitch/roll/yaw to achieve a Helix

# Make this program exacutable
# chmod +x nodes/HelixTrajectoryController.py

import rospy
import time
import numpy as np
#import tf  # use tf to easily calculate orientations


from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

global_pos= Twist()#np.array([0.,0.,0.])
global_zeropos= Twist()#np.array([0.,0.,0.])
global_command= Twist()
global_vel=Twist()
global_waypoint=Point()

islanded=True

pub_commands= rospy.Publisher('bebop/cmd_vel',Twist,queue_size=1)
pub_takeoff= rospy.Publisher('bebop/takeoff',Empty,queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1) #trigger with pub_land.publish()
pub_waypoints= rospy.Publisher("/bebop/waypoint_ilya", Point, queue_size=1)

def quat_mult(a,b):
    
    c = np.array([0.,0.,0.,0.])
    c[0] = (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3] )
    c[1] = (a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2] )
    c[2] = (a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1] )
    c[3] = (a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0] )
    return c

def callback(msg):
    global global_pos
    global global_vel
    rospy.loginfo(msg.pose.pose)
    rospy.loginfo(msg.twist.twist)

    #global_pos=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
    #print(global_pos)
    global_pos=msg.pose.pose
    global_vel=msg.twist.twist

def setlanded(msg):
    global islanded
    islanded=True
    pub_land.publish()

def odomlistener():
    #okay so we start this listener node
    global global_zeropos
    global global_pos
    global islanded

    rospy.init_node('odomlistener_move', anonymous=True, log_level=rospy.WARN)
    rospy.Subscriber('/bebop/odom', Odometry, callback)
    rospy.Subscriber('/bebop/land',Empty, setlanded)
    print('node started')
    time.sleep(2.)
    print('taking off')
    pub_takeoff.publish()
    time.sleep(6.)
    islanded=False
    print('setting zero point')
    global_zeropos=global_pos
    print(global_zeropos)

    print('lowering global zero point')
    global_zeropos.position.z= global_zeropos.position.z-.2
    print(global_zeropos)

    telemrate = 10
    rate = rospy.Rate(telemrate)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        
        # moveto(0,0,0)
        # moveto(0,0,1) 
        # moveto(1,1,1)   
        # moveto(1,1,2)
        # moveto(2,2,2)
        # moveto(2,2,3)
        # moveto(3,3,3)


        moveto(1,1,0)
        moveto(1,1,1)
        moveto(2,2,1)
        moveto(2,2,2)
        # moveto(3,3,2)
        # moveto(0,0,0)
        rate.sleep()	


#-0.34455513  1.90215293  1.39334602]

def moveto(x,y,z):
    global global_pos
    global global_vel
    global global_zeropos
    global global_command
    global islanded
    global global_waypoint


    #okay so have movement vector in body frame (relative to global_zeropos, want to figure out what it is in inertial frame
    #so you know your frame is rotated by quaternion in global_zeropos so want to rotate by the inverse
    quat_B_to_I= np.array([global_zeropos.orientation.w, -global_zeropos.orientation.x, -global_zeropos.orientation.y, -global_zeropos.orientation.z])
    quat_B_to_I_inv= np.array([global_zeropos.orientation.w, global_zeropos.orientation.x, global_zeropos.orientation.y, global_zeropos.orientation.z])
    command_quat_body=np.array([0, x,y,z])
    temp= quat_mult(quat_B_to_I_inv,command_quat_body)
    command_quat_inertial= quat_mult(temp,quat_B_to_I)
    command_vect_inertial= command_quat_inertial[1:]

    expected_pos_inertial= np.array([global_zeropos.position.x, global_zeropos.position.y, global_zeropos.position.z]) + command_vect_inertial
    global_waypoint.x=expected_pos_inertial[0]
    global_waypoint.y=expected_pos_inertial[1]
    global_waypoint.z=expected_pos_inertial[2]

    move_array=np.array([0.,0.,0.])
    #print('STARTING MOVE')
    if islanded==False:
        print('STARTING MOVE')
        print(np.array([x,y,z]))

        error=1000.
        error_integral=np.array([0.,0.,0.])
        while error>.15 and islanded==False:# and np.linalg.norm(np.array([global_vel.linear.x, global_vel.linear.y, global_vel.linear.y]))>.1:
            print('error-------------------------------------------------------------------------')
            print(error)
            current_pos_inertial=np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])
            move_vect_inertial= expected_pos_inertial-current_pos_inertial

            #okay need to convert to vector in body frame to figure out where to move
            #using current orientation global_pos in case something is flukey
            quat_I_to_B= np.array([global_pos.orientation.w, global_pos.orientation.x, global_pos.orientation.y, global_zeropos.orientation.z])
            quat_I_to_B_inv= np.array([quat_I_to_B[0], -quat_I_to_B[1], -quat_I_to_B[2],-quat_I_to_B[3]])
            
            move_quat_inertial= np.array([0, move_vect_inertial[0],  move_vect_inertial[1], move_vect_inertial[2]])
            temp= quat_mult(quat_I_to_B_inv,move_quat_inertial)
            move_quat_body= quat_mult(temp,quat_I_to_B)
            move_vect_body= move_quat_body[1:]# this is basically your error vector


            #HOLD UP THE VELOCITY IS IN "CHILD FRAME" WHICH IS SOME OTHER BULLSHIT from HEADER FRAME
            # #we also have 
            # velocity_quat_inertial= np.array([0., global_vel.linear.x, global_vel.linear.y,global_vel.linear.z])
            # #so lets put it in body frame
            # temp2= quat_mult(quat_I_to_B_inv,velocity_quat_inertial)
            # velocity_quat_body= quat_mult(temp2,quat_I_to_B)
            # velocity_vect_body= velocity_quat_body[1:]# this is basically your error vector derivitive
            #velocity_vect_body= np.array([global_vel.linear.x, global_vel.linear.y,global_vel.linear.z])
            velocity_vect_body= np.array([global_vel.linear.x, global_vel.linear.y, global_vel.linear.z])
            error_integral=error_integral+move_vect_body


            #move_vect_body[2]=1.29*move_vect_body[2]

            move_array[0]=.08*move_vect_body[0] - .16*velocity_vect_body[0] + .001*error_integral[0] #TUNE THIS
            move_array[1]=.08*move_vect_body[1] - .16*velocity_vect_body[1] + .001*error_integral[1]
            move_array[2]=.53*move_vect_body[2] - .10*velocity_vect_body[2] + .001*error_integral[2]


            timedelay= .1#TUNE THIS
            print('move vect')
            print(move_array)
            print('move_vect_body')
            print(move_vect_body)
            print('velocity_vect_body')
            print(velocity_vect_body)
            print('error_integral')
            print(error_integral)
            print(' ')
            print('command is')
            print(np.array([x,y,z]))
            print('expected_pos_inertial')
            print(expected_pos_inertial)
            print('current_pos_inertial')
            print(current_pos_inertial)
            print('global_zeropos')
            print(global_zeropos)



            global_command.linear.x=move_array[0]
            global_command.linear.y=move_array[1]
            global_command.linear.z=move_array[2]
            global_command.angular.x=0
            global_command.angular.y=0
            global_command.angular.z=0

            if rospy.is_shutdown():
                break

            pub_commands.publish(global_command)
            time.sleep(timedelay)
            # pub_commands.publish(global_command)
            # time.sleep(timedelay)
            # pub_commands.publish(global_command)
            # time.sleep(timedelay)
            error= np.linalg.norm(np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])-expected_pos_inertial)
            #error=.4
            pub_waypoints.publish(global_waypoint)

        #print('TRYING TO LAND! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        # pub_land.publish()
        # islanded=True
    else:
        print('Landed')
        pub_land.publish()
        islanded=True
    


    

if __name__ == '__main__':

    odomlistener()
    # rospy.spin()
