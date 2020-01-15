#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# Cleanly shutdown
import atexit

# ROS imports
import roslib, rospy

# numpy imports - basic math and matrix manipulation
import numpy as np

# message imports specific to this package
# from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32
# Phidgets stuff
import sys
import time 
from Phidget22.Devices.Stepper import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

################################################################################
# This function is copied verbatum from here: 
#      https://www.phidgets.com/?view=code_samples&lang=Python

def get_stepper_motor():
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
        
    return ch # this is a stepper motor object. API: https://www.phidgets.com/?view=api (select python & Stepper)

################################################################################

class Stepper_Motor_Velocity_Controller:
    def __init__(self, topic):
        # Define the source of the velocity control
        self.topic = topic

        # Velocity control subscriber
        self.velocity_control_sub = rospy.Subscriber("mag_switch", Int8MultiArray, self.velocity_changed_callback)
        # self.velocity_control_sub2 = rospy.Subscriber("mag_switch", Int8MultiArray, self.velocity_changed_callback2)

        self.pos_sub = rospy.Subscriber(self.topic, Float32, self.pos_callback)
        # self.pos_pub = rospy.Publisher(topic, Float32, queue_size=100)
        # self.POSPUB=rosp
        self.publisher = rospy.Publisher("motor_1", Float32, queue_size=100)
        # Set up the Stepper Motor
        self.stepper_motor = get_stepper_motor()
        self.stepper_motor.setControlMode(0) # velocity control (0 = position control)
        self.stepper_motor.setEngaged(1)
        self.state=None
        self.P=None




    def pos_callback(self, msg):
        self.P=msg.data
        print(self.P)

    def velocity_changed_callback(self, data):
        #  Set the velocity
        # i=0
        if self.state != data.data:
            self.state=data.data
            print(self.state)
            # time.sleep(.5)
        # if data.data[5] == 0 or data.data[3] == 0 :
        #     self.stepper_motor.setTargetPosition(0)
        #     self.stepper_motor.setVelocityLimit(6000)
        # else:
        #     self.stepper_motor.setTargetPosition(50000)
        #     self.stepper_motor.setVelocityLimit(2500)

        # print(pos)


     #this one 




                # if pos >= 5000:
                #     self.stepper_motor.setTargetPosition(10000)
                #     self.stepper_motor.setVelocityLimit(2500)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)
                # if pos >= 10000:
                #     self.stepper_motor.setTargetPosition(20000)
                #     self.stepper_motor.setVelocityLimit(2500)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)
                # if pos == 20000:
                #     self.stepper_motor.setTargetPosition(pos)
                #     self.stepper_motor.setVelocityLimit(0)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)        
            
    def main(self):
        # atexit.register(self.shutdown_stepper)
        # try:
        #     rospy.spin()
        # except KeyboardInterrupt:
        #     print "Shutting down"

        atexit.register(self.shutdown_stepper)

        rate = rospy.Rate(40) # Hz
        while not rospy.is_shutdown():
            if self.state is not None and self.P is not None:
                if self.P % 2 == 0:
                    self.stepper_motor.setTargetPosition(self.P)
                    self.stepper_motor.setVelocityLimit(1000)
                    pos=self.stepper_motor.getPosition()
                    print(pos)
                    if self.state[3] == 0 and self.state[5] == 1:
                        self.stepper_motor.setTargetPosition(0)
                        self.stepper_motor.setVelocityLimit(1000)
                        pos=self.stepper_motor.getPosition()                   
                        print(pos)
                    # self.publisher.publish(pos)
                    elif self.state[3] == 1 and self.state[5] == 0:                  
                        self.stepper_motor.setTargetPosition(0)
                        self.stepper_motor.setVelocityLimit(1000)
                        pos=self.stepper_motor.getPosition()
                        print(pos)
                    # self.publisher.publish(pos)
                elif self.P % 2 != 0:
                    self.stepper_motor.setTargetPosition(-1*self.P)
                    self.stepper_motor.setVelocityLimit(1000)
                    pos=self.stepper_motor.getPosition()
                    print(pos)
                    if self.state[3] == 0 and self.state[5] == 1:
                        self.stepper_motor.setTargetPosition(0)
                        self.stepper_motor.setVelocityLimit(1000)
                        pos=self.stepper_motor.getPosition()                   
                        print(pos)
                    # self.publisher.publish(pos)
                    elif self.state[3] == 1 and self.state[5] == 0:                  
                        self.stepper_motor.setTargetPosition(0)
                        self.stepper_motor.setVelocityLimit(1000)
                        pos=self.stepper_motor.getPosition()
                        print(pos)
                    # self.publisher.publish(pos)
                # elif self.state[3] == 0 and self.state[5] == 1:
                #     self.stepper_motor.setTargetPosition(self.P)
                #     self.stepper_motor.setVelocityLimit(0)
                #     pos=self.stepper_motor.getPosition()                   
                #     print(pos)
                #     # self.publisher.publish(pos)
                # elif self.state[3] == 1 and self.state[5] == 0:                  
                #     self.stepper_motor.setTargetPosition(self.P)
                #     self.stepper_motor.setVelocityLimit(0)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)
                    # self.publisher.publish(pos)
                pos=self.stepper_motor.getPosition()
                self.publisher.publish(pos)

                    
                # elif self.state[5] == 1 and self.state[3] == 0:
                #     self.stepper_motor.setTargetPosition(-1*self.P)
                #     self.stepper_motor.setVelocityLimit(250)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)
                # elif self.state[3] == 1 and self.state[5] == 0:
                #     self.stepper_motor.setTargetPosition(self.P)
                #     self.stepper_motor.setVelocityLimit(250)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)
                # elif self.state[3] == 1 and self.state[5] ==1:
                #     self.stepper_motor.setTargetPosition(0)
                #     self.stepper_motor.setVelocityLimit(250)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)

                # good code
                # if self.state[5] == 0 or self.state[3] == 0 :
                #     self.stepper_motor.setTargetPosition(0)
                #     self.stepper_motor.setVelocityLimit(6000)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)
                #     # pos=self.stepper_motor.getPosition()
                #     # print(pos)
                #     # self.stepper_motor.setTargetPosition(pos)
                #     # self.stepper_motor.setVelocityLimit(0)
                    
                # else:
                #     self.stepper_motor.setTargetPosition(-5000)
                #     self.stepper_motor.setVelocityLimit(2500)
                #     pos=self.stepper_motor.getPosition()
                #     print(pos)


                    #time.sleep(3)
                    #self.stepper_motor.setTargetPosition(10000)
                    #self.stepper_motor.setVelocityLimit(2500)
                    #pos=self.stepper_motor.getPosition()
                    #print(pos)


            # self.publisher.publish(pos)

            rate.sleep()

    def shutdown_stepper(self):
        self.stepper_motor.setEngaged(0)
        print("Stopped Stepper device")

################################################################################

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='',
                        help="ros topic with Float32 message for velocity control")
    (options, args) = parser.parse_args()



    rospy.init_node('stepper_motor', anonymous=True)

    stepper = Stepper_Motor_Velocity_Controller(options.topic)
    stepper.main()