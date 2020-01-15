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

try:
    from PhidgetHelperFunctions import *
except ImportError:
    sys.stderr.write("\nCould not find PhidgetHelperFunctions. Either add PhdiegtHelperFunctions.py to your project folder "
                      "or remove the import from your project.")
    sys.stderr.write("\nPress ENTER to end program.")
    readin = sys.stdin.readline()
    sys.exit()

############################################################################################
def onAttachHandler(self):
    ph = self
    try:
        #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
        #www.phidgets.com/docs/Using_Multiple_Phidgets for information
        print("\nAttach Event:")
        """
        * Get device information and display it.
        """
        serialNumber = ph.getDeviceSerialNumber()
        channelClass = ph.getChannelClassName()
        channel = ph.getChannel()
        
        deviceClass = ph.getDeviceClass()
        if (deviceClass != DeviceClass.PHIDCLASS_VINT):
            print("\n\t-> Channel Class: " + channelClass + "\n\t-> Serial Number: " + str(serialNumber) +
                  "\n\t-> Channel:  " + str(channel) + "\n")
        else:            
            hubPort = ph.getHubPort()
            print("\n\t-> Channel Class: " + channelClass + "\n\t-> Serial Number: " + str(serialNumber) +
                  "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
        
    except PhidgetException as e:
        print("\nError in Attach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

def onDetachHandler(self):
        ph = self
        try:
            #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
            #www.phidgets.com/docs/Using_Multiple_Phidgets for information
            print("\nDetach Event:")
            """
            * Get device information and display it.
            """
            serialNumber = ph.getDeviceSerialNumber()
            channelClass = ph.getChannelClassName()
            channel = ph.getChannel()
            
            deviceClass = ph.getDeviceClass()
            if (deviceClass != DeviceClass.PHIDCLASS_VINT):
                print("\n\t-> Channel Class: " + channelClass + "\n\t-> Serial Number: " + str(serialNumber) +
                      "\n\t-> Channel:  " + str(channel) + "\n")
            else:            
                hubPort = ph.getHubPort()
                print("\n\t-> Channel Class: " + channelClass + "\n\t-> Serial Number: " + str(serialNumber) +
                      "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
            
        except PhidgetException as e:
            print("\nError in Detach Event:")
            DisplayError(e)
            traceback.print_exc()
            return

def onErrorHandler(self,errorCode, errorString):
        sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ") \n")

def setup_phidget(self, port, serialNumber=559177, channel=0):
    ch = DigitalInput()        
    ch.setDeviceSerialNumber(serialNumber)
    ch.setHubPort(port)
    ch.setChannel(channel)
    ch.setIsHubPortDevice(True)
    ch.setOnAttachHandler(onAttachHandler)
    ch.setOnDetachHandler(onDetachHandler)
    ch.setOnErrorHandler(onErrorHandler)
    ch.setOnStateChangeHandler(self.onStateChangeHandler)
    return ch
############################################################################################

class Mag_input(object):
    def __init__(self, port_numbers, port="/dev/vinthub", topic='/mag_input', ):
        '''
        port_numbers: list of vint port numbers with mag switches attached
        '''
        print('Connecting to: ', port)
        print('And port numbers: ', port_numbers)

        self.state = [-1 for i in range(6)]

        self.publisher = rospy.Publisher(topic, Int8MultiArray, queue_size=10)
        self.setup_all_phidget_vint_ports(port_numbers)

    def setup_all_phidget_vint_ports(self, port_numbers):
        self.phidget_vint_objects = []
        for port in port_numbers:
            ch = setup_phidget(self, port)
            try:
                ch.openWaitForAttachment(5000)
            except PhidgetException as e:
                PrintOpenErrorMessage(e, ch)
                raise EndProgramSignal("Program Terminated: Open Failed")
            self.phidget_vint_objects.append(ch)

    def onStateChangeHandler(self, phidget_vint_port, state):
        port = phidget_vint_port.getHubPort()
        self.state[port] = state
        print(self.state)

    def main(self):

        rate = rospy.Rate(40) # Hz
        while not rospy.is_shutdown():
            #if self.state is not None:
            #    msg = Bool()
            #    msg.data=self.state
            #    self.publisher.publish(msg)
            
            msg = Int8MultiArray()
            msg.data = [self.state[0],self.state[1],self.state[2],self.state[3],self.state[4],self.state[5]]
            self.publisher.publish(msg)

            rate.sleep()



if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--port", type="str", dest="port", default='/dev/vinthub',
                        help="port to which HUB0000_0 is connected")
    #parser.add_option("--topic", type="str", dest="topic", default='/mag_input',
    #                    help="rostopic to publish to")
    (options, args) = parser.parse_args()

    port_numbers = [3,5]

    rospy.init_node('mag_switch', anonymous=True)
    topic = 'mag_switch'

    mag_input = Mag_input(port_numbers, port=options.port, topic=topic)
    mag_input.main()


