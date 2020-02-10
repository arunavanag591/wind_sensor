#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# Cleanly shutdown
import atexit

# ROS imports
import roslib, rospy
import serial
from std_msgs.msg import Int8MultiArray

from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.Stepper import *
import traceback
import time
import sys


def get_stepper_motor(serial):
    try:
        ch = Stepper()
    except RuntimeError as e:
        print("Runtime Exception %s" % e.details)
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)

    def StepperAttached(self):
        try:
            attached = self
            print("\nAttach Event Detected (Information Below)")
            print("===========================================")
            print("Library Version: %s" % attached.getLibraryVersion())
            print("Serial Number: %d" % attached.getDeviceSerialNumber())
            print("Channel: %d" % attached.getChannel())
            print("Channel Class: %s" % attached.getChannelClass())
            print("Channel Name: %s" % attached.getChannelName())
            print("Device ID: %d" % attached.getDeviceID())
            print("Device Version: %d" % attached.getDeviceVersion())
            print("Device Name: %s" % attached.getDeviceName())
            print("Device Class: %d" % attached.getDeviceClass())
            print("\n")

        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Press Enter to Exit...\n")
            readin = sys.stdin.read(1)
            exit(1)   
        
    def StepperDetached(self):
        detached = self
        try:
            print("\nDetach event on Port %d Channel %d" % (detached.getHubPort(), detached.getChannel()))
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Press Enter to Exit...\n")
            readin = sys.stdin.read(1)
            exit(1)   

    def ErrorEvent(self, eCode, description):
        print("Error %i : %s" % (eCode, description))

    def PositionChangeHandler(self, position):
        print("Position: %f" % position)

    try:
        ch.setDeviceSerialNumber(serial)
        ch.setOnAttachHandler(StepperAttached)
        ch.setOnDetachHandler(StepperDetached)
        ch.setOnErrorHandler(ErrorEvent)

        print("Waiting for the Phidget Stepper Object to be attached...")
        ch.openWaitForAttachment(5000)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)
        
    return ch 



class Stepper_Control:
  def __init__(self, ch):
    
    # Set up the Stepper Motor
    self.ch0 = ch
    
    rospy.Subscriber("mag_switch", Int8MultiArray, self.limit_switch)  

    self.ch0_position = 0
    self.limit_switch_0 = 0
    self.limit_switch_1 = 0
    self.limit_switch_2 = 0
    self.limit_switch_3 = 0
    

    atexit.register(self.shutdown_stepper)
    

################################################################################

  def limit_switch(self, data):
    self.limit_switch_0 = data.data[0]
    self.limit_switch_1 = data.data[1]
    self.limit_switch_2 = data.data[2]
    self.limit_switch_3 = data.data[3]
        

  def move_motor_safely_towards_0(self, desired_position):
    increment = 10
    while self.limit_switch_0 == 0 and self.ch0_position < desired_position:
      self.ch0.setTargetPosition(self.ch0_position + increment)
      self.ch0_position += increment
      rospy.sleep(0.05)
      

  def move_motor_safely_towards_1(self, desired_position):
  
    increment = -10
    while self.limit_switch_1 == 0 and self.ch0_position > desired_position:
      self.ch0.setTargetPosition(self.ch0_position + increment)
      self.ch0_position += increment
      rospy.sleep(0.05)

  def move_motor_safely_towards_2(self, desired_position):
    print (desired_position)
    increment = 100
    while self.limit_switch_2 == 0 and self.ch0_position < desired_position:
      self.ch0.setTargetPosition(self.ch0_position + increment)
      self.ch0_position += increment
      rospy.sleep(0.05)

  def move_motor_safely_towards_3(self, desired_position):    
    increment = -100
    while self.limit_switch_3 == 0 and self.ch0_position > desired_position:
      self.ch0.setTargetPosition(self.ch0_position + increment)
      self.ch0_position += increment
      rospy.sleep(0.05)



  def move_motor_towards_1(self):
  
    try:
        increment = 500
        
        if self.limit_switch_2 == 0:
          print (self.limit_switch_2)
          self.move_motor_safely_towards_2(self.ch0_position+increment)
          rospy.sleep(2)
        
        print ("loopexited")
          

    except PhidgetException as ex:
        #We will catch Phidget Exceptions here, and print the error informaiton.
        traceback.print_exc()
        print("")
        print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)

  def shutdown_stepper(self):
    self.ch0.setEngaged(0)
    self.ch0.close()    
    print("Stopped Stepper device")