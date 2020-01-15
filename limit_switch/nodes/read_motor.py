#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
from optparse import OptionParser
import sys
import time
import traceback
from Phidget22.Devices.DigitalInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

class Motor_Data:
    def __init__ (self, topic):
        rospy.Subscriber("motor_1", Float32, self.callback)
        rospy.Subscriber("mag_switch", Int8MultiArray, self.callback2)
        self.pubmax=rospy.Publisher("max",Float32,queue_size=100)
        self.pubmin=rospy.Publisher("min",Float32,queue_size=100)
        self.position=None
        self.state=None
        self.last_pos=-100000000000
        self.is_stop=False




    def callback(self,dat):
        self.position=dat.data
        if self.position == self.last_pos:
            self.is_stop=True
        else:
            self.last_pos=self.position



    def callback2(self,data):
        if self.state != data.data:
            self.state=data.data
            # print(self.state)

    def main(self):
        rate = rospy.Rate(40) # Hz
        while not rospy.is_shutdown():
            if self.state is not None and self.last_pos is not None:
                if self.state[5] == 1 and self.is_stop == True:
                    print("this is the max " + str(self.last_pos))
                    Max=self.last_pos
                    self.pubmax.publish(Max)
                elif self.state[3]==1 and self.is_stop == True:
                    print("this is the min" + str(self.last_pos))
                    Min=self.last_pos
                    self.pubmin.publish(Min)
                else:
                    # self.state[5] == 0 and self.state[3] == 0 and self.is_stop == True:
                    print("Didnt get to the end go more" + str(self.last_pos))
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('read_motor', anonymous=True)
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='',
                    help="ros topic with Float32 message for velocity control")
    (options, args) = parser.parse_args()
    motor = Motor_Data(options.topic)
    motor.main()