#!/usr/bin/env python


## Helical Trajectory Commander that listens to Bebop odometry messages
## And commands bebop pitch/roll/yaw to achieve a Helix

# Make this program exacutable
# chmod +x nodes/HelixTrajectoryController.py

import rospy
import numpy as np
import tf  # use tf to easily calculate orientations


from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
class Command:
    def __init__(self):
        self.cmd_pos = [0,0,0]
        self.cmd_vel = [0,0,0]
        self.time = 0
        self.time_to_complete = 0 # time to complete commanded trajectory
        self.cmdTime = 0 # initialize command start time to zero
        self.Kp_cmd = 0.1 # Initialize proportional position error gain
        self.Kd_cmd = 0.1 # Initialize derivative velocity error gain


    def generateHelixTrajectory(self):
           # Initialize Helix Trajectory
        radius = 1 # 1 meter radius
        z_up = 1 # i meter pitch per turn
        time_to_complete = 5 # 5 seconds to complete 1 turn
        telemrate = 5
        ang_vel = 2*3.14159/time_to_complete
        circumf = 2*3.14159*radius
        linear_vel = circumf/time_to_complete # Calculater constant velocity along circle
        # Assume ENU coordinate frame with takeoff point being (0,0,0)
        cmdLen = telemrate*time_to_complete # sets length of command position vector assuming the telemetry rate is held constant
        cmd_pos = np.zeros([3,cmdLen])
        cmd_vel = np.zeros([3,cmdLen])
        cmd_angular_pos = np.zeros([1,cmdLen])
        time = np.zeros([1,cmdLen]) # time from command start

        # generate command position and velocities along the circle
        i=0
        for i in range(cmdLen):
            cmd_angular_pos[:,i] = i/cmdLen*(2*3.14159) #Fraction of one full rotation
            cmd_pos[:,i] = np.concatenate([radius*np.cos(cmd_angular_pos[:,i]),radius*np.sin(cmd_angular_pos[:,i]),np.array([z_up*i/cmdLen])])
            cmd_vel[:,i] = np.concatenate([-linear_vel*np.sin(cmd_angular_pos[:,i]),linear_vel*np.cos(cmd_angular_pos[:,i]),np.array([z_up/time_to_complete])]) #m/s
            time[:,i] = i/cmdLen*time_to_complete

        self.cmd_pos = cmd_pos
        self.cmd_vel = cmd_vel
        self.time_to_complete = time_to_complete
        self.time = time
        self.cmdTime = 0 # initialize command start time to zero

        return cmd

def callback(self, data, cmd):

    # Initialize publisher to publish to the "cmd_vel" topic which is the Bebop pitch/roll/yaw command line
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10) # set output message to "Twist" which is the expected message type for Bebop commands 
    rospy.init_node('trajectory_commander', anonymous=True)
    telemrate = 5
    rate = rospy.Rate(telemrate) # Set trajectory controller freq to 5hz - this is due to limitations of Bebop odom telemetry which is limited to 5 hz max
    rospy.loginfo(data) # Logs telemetry for future use/debuggiong


    # Parse data into position vectors in the normal and binormal directions to the commanded trajectory
    # First, read Odom message as position (x,y,z) and Orientation (x,y,z,w) quaternions and Velocity East,North,Up
    timenow = odom.header.stamp
    if cmd.cmdTime == 0:
        cmd.cmdTime = timenow #start the command trajectory NOW

    pos = [data.pose.pose.position.x, # m Position from takeoff.  Positive East
        data.pose.pose.position.y, # m Position from takeoff.  Positive North
        data.pose.pose.position.z] # m Position from takeoff.  Positive Up
    ori = [data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w]
    # Transform quaternions to euler angle
    oriZYX = tf.transformations.euler_from_quaternion(ori)

    vel = [data.twist.twist.linear.x, #m/s Velocity.  Positive East
        data.twist.twist.linear.y, #m/s Velocity.  Positive North
        data.twist.twist.linear.z] #m/s Velocity.  Positive Up
    # Ok, now transform this position along the trajectory normal and binorml direction
    # normal = +left?
    # binormal = +up?

    deltaT = cmd.cmdTime-timenow #calculate how far into command we are
    if daltaT>cmd.time_to_complete:
        # Command HOVER - all zeros and put commands in Twist Message form
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
    else:
        # Interpolate command arrays to find desired position and velocity
        cmdIdx = np.find_nearest(self.cmd.time, deltaT)
        cmd_pos = cmd.cmd_pos[cmdIdx,:]
        cmd_vel = cmd.cmd_vel[cmdIdx,:]
        tangent = cmd_vel/np/linalg.norm(cmd_vel)
        #normal = 
        #binormal = [,,]

        # Why not just use x and y errors in the body frame instead of normal/binormal errors?  This would be easier to impliment

        # Calculate normal error (cross, left/right error)
        poserr_norm = 0 # equation here
        velerr_norm = 0 # equation here
        # Calculate binormal Z error (glide, up/down error)
        poserr_binorm = 0 # equation here
        velerr_binorm = 0 # equation here
        # Program controller Here
        # 
        r1accel_des = cmd.Kp_cmd*poserr_norm + cmd.Kd_cmd*velerr_norm
        r2accel_des = cmd.Kp_cmd*poserr_binorm + cmd.Kd_cmd*velerr_binorm

        pitch_angle = 0 # Calculate this based on desired accelerations in the body frame
        roll_angle = 0 # Calculate this based on desired accelerations in the body frame


        # put commands in Twist Message form.  This is the way commands are actually taken by the bebop
        #    roll_degree       = linear.y  * max_tilt_angle
        #    pitch_degree      = linear.x  * max_tilt_angle
        #    ver_vel_m_per_s   = linear.z  * max_vert_speed
        #    rot_vel_deg_per_s = angular.z * max_rot_speed
        vel_msg = Twist()
        vel_msg.linear.x = pitch_angle/max_tilt_angle # Pitch
        vel_msg.linear.y = roll_angle/max_tilt_angle # Roll
        vel_msg.linear.z = 0 # Climb/descend
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0 # Yawrate

    pub.publish(vel_msg)
    rate.sleep()

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('trajectory_commander', Odometry, callback,cmd) # Set up the listener to look for Odometry messages

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cmd = Command() # Initialize command object
    cmd.generateHelixTrajectory() # generate trajectory
    listener()

