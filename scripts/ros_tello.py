#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# tello_control.py
#
# Created on: Dec 16, 2018
#     Author: Yu Okamoto
# Brief: tello ros driver using dji-sdk

# import math
# import numpy as np
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from tellolib.Tello_Video.tello import Tello
from tellolib.Tello_Video.tello_control_ui import TelloUI

import time
import threading
# from PIL import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

loop_freq = 0.2
loop_dt = 5.0


class Mode(object):
    LANDED = 0
    FLYING = 1
    UNKNOWN = 2

class TelloROSDriver(object):
    ''' sub scribe topic and publish after safety check
    '''

    def __init__(self, tello):

        self._tello = tello
        self._cmd_vel = Twist()
        self._mode = Mode.LANDED
        self.bridge = CvBridge()
        
        self._img_pub = rospy.Publisher('image_raw', Image, queue_size=10)
        self._cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_sub_cb)
        self._mode_sub = rospy.Subscriber('mode', Int8, self._mode_sub_cb)

        # start a thread that constantly pools the video sensor for
        # the most recently read frame
        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread.start()
    
    def videoLoop(self):
        """
        COPY and modified from tello_control_ui.py in Tello-Python
        The mainloop thread of Tkinter 
        Raises:
            RuntimeError: To get around a RunTime error that Tkinter throws due to threading.
        """
        try:
            # start the thread that get GUI image and drwa skeleton 
            time.sleep(0.5)
            while not self.stopEvent.is_set():                
                # read the frame for GUI show
                self.frame = self._tello.read()
                if self.frame is None or self.frame.size == 0:
                    continue           

                # smoothing filter
                self.frame = cv2.bilateralFilter(self.frame, 5, 50, 100)  
                self._img_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "rgb8"))
                time.sleep(0.045) #todo need this sleep to avoid block noize

        except RuntimeError, e:
            print("[INFO] caught a RuntimeError")

    def _cmd_vel_sub_cb(self, msg):
        self._cmd_vel_sub = msg
        if msg.linear.x != 0.0:
            self._tello.move_forward()

    def _mode_sub_cb(self, msg):

        self._mode = msg.data
        self._cmd_vel = Twist()

        if self._mode == Mode.LANDED:
            self._tello.land()
        elif self._mode == Mode.FLYING:
            self._tello.takeoff()

    def spin(self):
        r = rospy.Rate(loop_freq)
        while not rospy.is_shutdown():
            self._tello.send_command('command')        
            if self._mode == Mode.FLYING:
                pass
                #command velocity
            else:
                pass
            r.sleep()

def main():
    tello = Tello('', 8889) 

    rospy.init_node('tello_control')
    node = TelloROSDriver(tello)
    node.spin()

if __name__ == '__main__':
    main()