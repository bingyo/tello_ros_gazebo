#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist, Quaternion, PointStamped, Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool, ColorRGBA, Empty
from std_srvs.srv import Trigger, TriggerResponse, SetBool
from hector_uav_msgs.srv import EnableMotors

import select, termios, tty


class ForestMotion:

    def init(self):

        rospy.init_node('automove', anonymous=True)

        self.task_start_ = False 
        self.odom_update_flag_ = False
        self.task_start_time_ = rospy.Time.now()

        #self.vel_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        #self.odom_sub_ = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.odomCallback)
        self.vel_pub_ = rospy.Publisher("/tello/cmd_vel", Twist, queue_size = 1)
        self.odom_sub_ = rospy.Subscriber("/tello/orb_slam2_mono/pose", PoseStamped, self.odomCallback)
        self.odom_sub_ = rospy.Subscriber("/tello/stop", Bool, self.stopCallback)
        self._mode_srv = rospy.ServiceProxy('/tello/enable_motors', EnableMotors)

        self.control_rate_ = 20
        self.state_x = 0
        self.state_y = 0
        self.state_z = 0
        self.state_x_init = 0
        self.state_y_init = 0
        self.state_z_init = 0
        self.state = 0
        self.VEL_SPEAD_ = 0.15
        self.DIS_ = 0.04
        
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

    def taskStartCallback(self, msg):
        rospy.loginfo("Task Start")
        print("start")
        self.task_start_time_ = rospy.Time.now()
        self.task_start_ = True
        #self.task_start_sub_.unregister()
    
    def odomCallback(self, msg):

        if not self.odom_update_flag_:
            self.state_x_init = msg.pose.position.x
            self.state_y_init = msg.pose.position.y
            self.state_z_init = msg.pose.position.z
            self.odom_update_flag_ = True
        #self.task_start_ = True

        self.state_x = msg.pose.position.x + self.state_x_init 
        self.state_y = msg.pose.position.y + self.state_y_init
        self.state_z = msg.pose.position.z
        print(self.state_x)

    def controlCallback(self, event):

        vel_msg = Twist()

        if (not self.odom_update_flag_) or (not self.task_start_):
            self._mode_srv(True)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            self.vel_pub_.publish(vel_msg)
            time.sleep(3.0)
            self.task_start_ = True
            return

        if self.state_z < 0.04:
            vel_msg.linear.z = self.VEL_SPEAD_
            #print("up")
        else:
            vel_msg.linear.z = 0.0
            #print("stop")

        if self.state == 0:
            vel_msg.linear.x = self.VEL_SPEAD_
            print('mae')
            if self.state_x > self.DIS_:
                self.state = 1

        elif self.state == 1:
            vel_msg.linear.x = -1*self.VEL_SPEAD_
            print('ushiro')
            if self.state_x < -1*self.DIS_:
                self.state = 2

        elif self.state == 2:
            vel_msg.linear.x = self.VEL_SPEAD_
            print('mae2')
            if self.state_x > 0.0:
                self.state = 3

        elif self.state == 3:
            vel_msg.linear.y = self.VEL_SPEAD_
            print('left')
            if self.state_y > self.DIS_:
                self.state = 4

        elif self.state == 4:
            vel_msg.linear.y = -1*self.VEL_SPEAD_
            print('right')
            if self.state_y < -1*self.DIS_:
                self.state = 5

        elif self.state == 5:
            vel_msg.linear.y = self.VEL_SPEAD_
            print('shoki')
            if self.state_y > 0.0:
                self.state = 0

        else:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            print('finish')
            self._mode_srv(False)
            self.task_start_ = False
            self.control_timer_.shutdown()
        

        self.vel_pub_.publish(vel_msg)

    def stopCallback(self, event):
        self.state = -1


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    try:
        forest_motion = ForestMotion()
        forest_motion.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
