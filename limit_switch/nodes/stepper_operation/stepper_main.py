#!/usr/bin/env python

import rospy
import stepper_class


if __name__ == '__main__':
  try:
    ch0 = stepper_class.get_stepper_motor(522470)
    ch0.setEngaged(True)
    #ch0.setDeviceSerialNumber(505962)
    

    #ch1 = stepper_class.get_stepper_motor()
    #ch0.setDeviceSerialNumber(522470)
    #ch1.setEngaged(True)

    rospy.init_node('stepper_motor', anonymous=True)

    stepper = stepper_class.Stepper_Control(ch0)
    stepper.move_motor_towards_1()

  except rospy.ROSInterruptException:
    pass


