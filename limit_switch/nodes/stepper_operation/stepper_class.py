#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# Cleanly shutdown
import atexit

# ROS imports
import roslib, rospy
import serial
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int64MultiArray
# from std_msgs.msg import Float32MultiArray

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
    # self.stepper_motor = get_stepper_motor0(505962)

    rospy.Subscriber("mag_switch", Int8MultiArray, self.limit_switch)
    self.publisher = rospy.Publisher('motopub', Int64MultiArray, queue_size=10)  

    self.ch0_position = 0
    self.ch1_position = 0
    self.limit_switch_0 = 0
    self.limit_switch_1 = 0
    self.limit_switch_2 = 0
    self.limit_switch_3 = 0
    

    atexit.register(self.shutdown_stepper)

    self.publish_motor_data()

################################################################################

  def limit_switch(self, data):
    self.limit_switch_0 = data.data[0]
    self.limit_switch_1 = data.data[1]
    self.limit_switch_2 = data.data[2]
    self.limit_switch_3 = data.data[3]
        
  def publish_motor_data(self):
    print(self.ch0_position, self.ch1_position)
    msg = Int64MultiArray()
    msg.data = [int(self.ch0_position), int(self.ch1_position)]
    self.publisher.publish(msg)

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
    increment1 = 1000
    increment2 = -1000 
    

    while self.limit_switch_0 == 0:
      self.move_motor_safely_towards_0(self.ch0_position+increment1)
      rospy.sleep(30)# change this timing to change the horizontal step gap duration
      self.publish_motor_data()
      # MES.data = rospy.Time.now().secs
      # self.publisher.publish(MES) 

    while self.limit_switch_1 == 0:
      self.move_motor_safely_towards_1(self.ch0_position+increment2)
      rospy.sleep(30)# change this timing to change the horizontal step gap duration
      # MES.data = 
      # self.publisher.publish(MES)
      self.publish_motor_data()


  def move_motor(self):
    # showtime()
    try:
        #vertical motor step size
        increment1 = 50000
        increment2 = -50000 
               
        for x in range(0,1):  #number of vertical cycles
          while self.limit_switch_2 == 0:
            self.move_motor_safely_towards_2(self.ch1_position+increment1)
            rospy.sleep(0.5) # change this timing to change moving down
            self.move_horizontal()
            rospy.sleep(.5) 
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

  # def showtime(self):

    # rate = rospy.Rate(40) # Hz
    # while not rospy.is_shutdown():
    #     #if self.state is not None:
    #     #    msg = Bool()
    #     #    msg.data=self.state
    #     #    self.publisher.publish(msg)
        
    #     msg  = Int8MultiArray()
    #     pos1 = int(self.ch0.getPosition())
    #     pos2 = int(self.ch1.getPosition())
    #     print("except11",[pos1,pos2, rospy.Time.now().secs])
        
    #   # MES.data=[pos1,pos2]
    #   # self.publisher.publish(MES)
    #     print([pos1,pos2,rospy.Time.now().secs])
    #     msg.data = [pos1,pos2,rospy.Time.now().secs] #,self.state[4],self.state[5]]
    #     self.publisher.publish(msg)

    #     rate.sleep()



