#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
import numpy as np
import math
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

        #self.vel_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) #for gazebo simulation
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
        self.VEL_SPEAD_ = 0.25
        self.DIS_ = 0.04

        self.count = 0
        self.time_tmp = 0
        self.x_tmp = 0
        self.y_tmp = 0
        self.p_x = 3.0
        self.p_y = 3.0
        self.d_x = 1.3
        self.d_y = 1.3
        
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

        self.state_x = msg.pose.position.x - self.state_x_init 
        self.state_y = msg.pose.position.y - self.state_y_init
        self.state_z = msg.pose.position.z
        #print(self.state_x)

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


        if (not self.odom_update_flag_) or (not self.task_start_):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            self.vel_pub_.publish(vel_msg)
            return

        #if self.state_z < 0.04:
        #    vel_msg.linear.z = self.VEL_SPEAD_
            #print("up")
        #else:
        #    vel_msg.linear.z = 0.0
            #print("stop")

        if self.state == 0:
            print('mae')
            vel_msg.linear.x = self.VEL_SPEAD_
            if self.state_x > self.DIS_:
                self.time_tmp = time.time()
                self.x_tmp = self.state_x
                self.y_tmp = self.state_y
                self.state = 1

        elif self.state == 1:
            #vel_msg.linear.x = self.VEL_SPEAD_ * math.cos(0.05*self.count)
            #vel_msg.linear.y = self.VEL_SPEAD_ * math.sin(0.05*self.count)
            self.count = self.count + 1
            #print('circle')

            x_c = 0.3 * math.cos(0.01*self.count)
            y_c = 0.3 * math.sin(0.01*self.count)

            now_x = self.state_x
            now_y = self.state_y

            time_diff = time.time() - self.time_tmp
            vel_msg.linear.x = self.p_x*(x_c-now_x)*self.VEL_SPEAD_ + self.d_x * ((x_c-now_x) - self.x_tmp)/ time_diff
            vel_msg.linear.y = self.p_y*(y_c-now_y)*self.VEL_SPEAD_ + self.d_y * ((y_c-now_y) - self.y_tmp)/ time_diff

            self.x_tmp = x_c-now_x
            self.y_tmp = y_c-now_y
            self.time_tmp = time.time()


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
