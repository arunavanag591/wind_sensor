#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Int8MultiArray
from optparse import OptionParser
import sys
import time
import traceback
from Phidget22.Devices.DigitalInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

def callback(data):
    rospy.loginfo("   I read %s", str(data.data[5]) +' on 5')
    if data.data[5] == 0:
    	print("NO5")
    else: 
		print("YE5")

def callback1(data):
    rospy.loginfo("   I read %s",str(data.data[3]) +' on 3')
    if data.data[3] == 0:
    	print("NO3")
    else: 
		for i in range (100):
			print("Y3S"+str(i))
			if i == 50:
				break


def listener():


    rospy.init_node('read_switch', anonymous=True)

    rospy.Subscriber("mag_switch", Int8MultiArray, callback)
    rospy.Subscriber("mag_switch", Int8MultiArray, callback1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()