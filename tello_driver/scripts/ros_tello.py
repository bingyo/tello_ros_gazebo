#!/usr/bin/env python

import math
# import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu, Range, BatteryState
from hector_uav_msgs.srv import EnableMotors

from tellolib.Tello_Video.tello import Tello
from tellolib.Tello_Video.tello_control_ui import TelloUI

import time
import threading
import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError

import ast

# copied from https://github.com/hanyazou/TelloPy
from protocol import *

LOOP_FREQ = 20
LOOP_DT = 0.05


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
        self._tof_pub = rospy.Publisher('tof_height', Range, queue_size=1)
        self._bat_pub = rospy.Publisher('battery', BatteryState, queue_size=1)
        self._cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_sub_cb)
        self._imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
        self._mode_srv = rospy.Service('enable_motors', EnableMotors, self._mode_srv_cb)

        # start a thread that constantly pools the video sensor for
        # the most recently read frame
        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread2 = threading.Thread(target=self.imuLoop, args=())
        self.thread.start()
        self.thread2.start()

    def imuLoop(self):
        t_tmp = time.time()
        angular_tmp = [0.0] * 3
        imuMsg = Imu()
        heightMsg = Range()
        batMsg = BatteryState()
        try:
            time.sleep(0.5)
            while not self.stopEvent.is_set():
                response = self._tello.get_response()
                if response is None: 
                    continue
                elif len(response) > 10:
                    response = response.strip(';\r\n')
                    response = response.replace(';', "','")
                    response = response.replace(':', "':'")
                    response = "{'" + response + "'}"
                    dic = ast.literal_eval(response)
                    if not 'agx' in dic:
                        continue
                    imuMsg.linear_acceleration.x = float(dic['agx']) / 102.0 #102=1000/9.8
                    imuMsg.linear_acceleration.y = float(dic['agy']) / 102.0
                    imuMsg.linear_acceleration.z = float(dic['agz']) / 102.0
                    dt = time.time() - t_tmp
                    t_tmp = time.time()
                    angular_roll = float(dic['roll'])
                    angular_pitch = float(dic['pitch'])
                    angular_yaw = float(dic['yaw'])
                    if (angular_yaw - angular_tmp[2]) > 180:#minus -> plus
                         angular_yaw = angular_yaw - 360.0
                    elif (angular_yaw - angular_tmp[2]) < -180:#plus -> minus
                         angular_yaw = angular_yaw + 360.0

                    imuMsg.angular_velocity.x = math.radians(angular_roll - angular_tmp[1]) / dt
                    imuMsg.angular_velocity.y = math.radians(angular_pitch - angular_tmp[0]) / dt
                    imuMsg.angular_velocity.z = math.radians(angular_yaw - angular_tmp[2]) / dt
                    angular_tmp[0] = float(dic['pitch'])
                    angular_tmp[1] = float(dic['roll'])
                    angular_tmp[2] = float(dic['yaw'])
                    imuMsg.header.stamp = rospy.Time.now()
                    imuMsg.header.frame_id = 'world'
                    ##imuMsg.orientation.w = 1
                    self._imu_pub.publish(imuMsg)

                    heightMsg.header.stamp = rospy.Time.now()
                    heightMsg.header.frame_id = 'world'
                    heightMsg.radiation_type = heightMsg.INFRARED
                    heightMsg.field_of_view = 0
                    heightMsg.max_range = 30
                    heightMsg.min_range = 0.3
                    heightMsg.range = int(dic['tof']) * 0.01 # cm
                    self._tof_pub.publish(heightMsg)

                    batMsg.header.stamp = rospy.Time.now()
                    batMsg.header.frame_id = 'world'
                    batMsg.voltage = 3.8
                    batMsg.design_capacity = 1.1
                    batMsg.percentage = float(dic['bat']) * 0.01
                    self._bat_pub.publish(batMsg)

                    time.sleep(0.004)

        except RuntimeError, e:
            rospy.loginfo('caught a RuntimeError for gettng IMU')


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
                #start = time.time()              
                # read the frame for GUI show
                self.frame = self._tello.read()
                if self.frame is None or self.frame.size == 0:
                    continue           

                # smoothing filter
                self.frame = cv2.bilateralFilter(self.frame, 5, 50, 100)  
                imgMsg = self.bridge.cv2_to_imgmsg(self.frame, "rgb8")
                imgMsg.header.stamp = rospy.Time.now()
                imgMsg.header.frame_id = "world"

                self._img_pub.publish(imgMsg)
                time.sleep(0.04)#todo cant find reason why quality less than tiker

        except RuntimeError, e:
            rospy.loginfo('caught a RuntimeError for gettng video')

    def _cmd_vel_sub_cb(self, msg):
        self._cmd_vel = msg

    def _mode_srv_cb(self, req):

        self._mode = req.enable
        self._cmd_vel = Twist()

        if self._mode == Mode.LANDED:
            self._tello.land()
            time.sleep(0.7)
            self._tello.land()
        elif self._mode == Mode.FLYING:
            self._tello.takeoff()

        return True


    def send_packet(self, pkt):
        """Send_packet is used to send a command packet to the drone."""
        try:
            cmd = pkt.get_buffer()
            self._tello.socket.sendto(cmd, self._tello.tello_address)
            rospy.logdebug("send_packet: %s" % byte_to_hexstring(cmd))
        except socket.error as err:
            if self.state == self.STATE_CONNECTED:
                rospy.logerr("send_packet: %s" % str(err))
            else:
                rospy.logerr("send_packet: %s" % str(err))
            return False

        return True

    def __send_stick_command(self, msg):
        '''
        copy and modified from Tellopy https://github.com/hanyazou/TelloPy
        '''
        fast_mode = False

        pkt = Packet(STICK_CMD, 0x60)

        axis1 = int(1024 + 660.0 * -msg.linear.y) & 0x7ff
        axis2 = int(1024 + 660.0 *  msg.linear.x) & 0x7ff
        axis3 = int(1024 + 660.0 *  msg.linear.z) & 0x7ff
        axis4 = int(1024 + 660.0 * -msg.angular.z) & 0x7ff
        axis5 = int(fast_mode) & 0x01
        rospy.logdebug("stick command: fast=%d yaw=%4d vrt=%4d pit=%4d rol=%4d" %
                       (axis5, axis4, axis3, axis2, axis1))

        '''
        11 bits (-1024 ~ +1023) x 4 axis = 44 bits
        fast_mode takes 1 bit
        44+1 bits will be packed in to 6 bytes (48 bits)

         axis5      axis4      axis3      axis2      axis1
             |          |          |          |          |
                 4         3         2         1         0
        98765432109876543210987654321098765432109876543210
         |       |       |       |       |       |       |
             byte5   byte4   byte3   byte2   byte1   byte0
        '''
        packed = axis1 | (axis2 << 11) | (
            axis3 << 22) | (axis4 << 33) | (axis5 << 44)
        packed_bytes = struct.pack('<Q', packed)
        pkt.add_byte(byte(packed_bytes[0]))
        pkt.add_byte(byte(packed_bytes[1]))
        pkt.add_byte(byte(packed_bytes[2]))
        pkt.add_byte(byte(packed_bytes[3]))
        pkt.add_byte(byte(packed_bytes[4]))
        pkt.add_byte(byte(packed_bytes[5]))
        pkt.add_time()
        pkt.fixup()
        rospy.logdebug("stick command: %s" %
                       byte_to_hexstring(pkt.get_buffer()))
        return self.send_packet(pkt)

    def spin(self):
        count = 0
        r = rospy.Rate(LOOP_FREQ)
        while not rospy.is_shutdown():
            if count > 100:
                count = 0
                self._tello.send_command('command')  
            if self._mode == Mode.FLYING:
                cmd = self._cmd_vel    
                self.__send_stick_command(cmd)
            else:
                pass

            count += 1
            r.sleep()
        
        self.stopEvent.set()
        self.thread.join()
        del self._tello

def main():
    tello = Tello('', 8890) 

    rospy.init_node('tello_control')
    node = TelloROSDriver(tello)
    node.spin()

if __name__ == '__main__':
    main()
