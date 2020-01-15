#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
from optparse import OptionParser
import sys
import time
import traceback
from numpy import *
import scipy.linalg
from Phidget22.Devices.DigitalInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

class Motor_Data:
    def __init__ (self, topic):
        rospy.Subscriber("max", Float32, self.callback)
        rospy.Subscriber("min", Float32, self.callback2)

        self.maximum=None
        self.minimum=None
        self.exstream_max=100000000000
        self.exstream_min=-100000000000
        self.is_stop1=False
        self.is_stop2=False




    def callback(self,dat):
        self.maximum=dat.data
        if self.maximum > self.exstream_max:
            self.is_stop1=True
        else:
            self.exstream_max=self.maximum
        # print(self.maximum)
        # print(self.is_stop1)



    def callback2(self,data):
        self.minimum=data.data
        if self.minimum < self.exstream_min:
            self.is_stop2=True
        else:
            self.exstream_min=self.minimum
        # print(self.is_stop2)

    def main(self):
        rate = rospy.Rate(40) # Hz
        while not rospy.is_shutdown():
            if self.exstream_max is not None or self.exstream_min is not None:
                # print("hello")
                if self.is_stop1 == True:
                    print("this is the right limit " + str(self.exstream_max))
                    Right=self.exstream_max
                elif self.is_stop2 == True:
                    print("this is the left limit " + str(self.exstream_min))
                    Left=self.exstream_min
                    # A=linspace(Left,Right,10)
                else:
                    print("no max no min yet")

            
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('read_motor', anonymous=True)
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='',
                    help="ros topic with Float32 message for velocity control")
    (options, args) = parser.parse_args()
    motor = Motor_Data(options.topic)
    motor.main()