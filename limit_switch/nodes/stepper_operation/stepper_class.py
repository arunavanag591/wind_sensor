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


def get_stepper_motor0(serial):
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


def get_stepper_motor1(serial):
    try:
        ch1 = Stepper()
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
        ch1.setDeviceSerialNumber(serial)
        ch1.setOnAttachHandler(StepperAttached)
        ch1.setOnDetachHandler(StepperDetached)
        ch1.setOnErrorHandler(ErrorEvent)

        print("Waiting for the Phidget Stepper Object to be attached...")
        ch1.openWaitForAttachment(5000)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)
        
    return ch1 


class Stepper_Control:
  def __init__(self, ch, ch1):
    
    # Set up the Stepper Motor
    self.ch0 = ch
    #self.ch0.setDeviceSerialNumber(505962)
    self.ch1 = ch1
    #self.ch0.setDeviceSerialNumber(522470)

    rospy.Subscriber("mag_switch", Int8MultiArray, self.limit_switch)  

    self.ch0_position = 0
    self.ch1_position = 0
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
        

  def move_motor_safely_towards_0(self, desired_position): #right
    increment = 10
    while self.limit_switch_0 == 0 and self.ch0_position < desired_position:
      self.ch0.setTargetPosition(self.ch0_position + increment)
      self.ch0_position += increment
      rospy.sleep(0.05)
      

  def move_motor_safely_towards_1(self, desired_position):  #left
    increment = -10
    while self.limit_switch_1 == 0 and self.ch0_position > desired_position:
      self.ch0.setTargetPosition(self.ch0_position + increment)
      self.ch0_position += increment
      rospy.sleep(0.05)

  def move_motor_safely_towards_2(self, desired_position): #up
    print (desired_position)
    increment = 100
    while self.limit_switch_2 == 0 and self.ch1_position < desired_position:
      self.ch1.setTargetPosition(self.ch1_position + increment)
      self.ch1_position += increment
      rospy.sleep(0.05)

  def move_motor_safely_towards_3(self, desired_position):  #down
    increment = -100
    while self.limit_switch_3 == 0 and self.ch1_position > desired_position:
      self.ch1.setTargetPosition(self.ch1_position + increment)
      self.ch1_position += increment
      rospy.sleep(0.05)

  def move_horizontal(self): #horizontalmovement
    #horizontal motor step size
    increment1 = 500
    increment2 = -500 

    while self.limit_switch_0 == 0:
      self.move_motor_safely_towards_0(self.ch0_position+increment1)
      rospy.sleep(0.5)

    while self.limit_switch_1 == 0:
      self.move_motor_safely_towards_1(self.ch0_position+increment2)
      rospy.sleep(0.5)
      

  def move_motor(self):
    try:
        #vertical motor step size
        increment1 = 50000
        increment2 = -50000 
               
        for x in range(0,1):  #number of vertical cycles
          while self.limit_switch_2 == 0:
            self.move_motor_safely_towards_2(self.ch1_position+increment1)
            rospy.sleep(0.5) # change this timing to change the gap between vertical and horizontal
            self.move_horizontal()
            rospy.sleep(0.5) # change this timing to change the horizontal step gap duration

          while self.limit_switch_3 == 0:
            self.move_motor_safely_towards_3(self.ch1_position+increment2)
            rospy.sleep(0.5)
            self.move_horizontal()
            rospy.sleep(0.5)

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