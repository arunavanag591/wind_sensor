#!/usr/bin/env python

import rospy
import stepper_class


if __name__ == '__main__':
  try:
    #Horizontal Motor - x
    ch0 = stepper_class.get_stepper_motor0(505962)
    ch0.setEngaged(True)
    #Vertical Motor - y
    ch1 = stepper_class.get_stepper_motor1(522470)
    ch1.setEngaged(True)

    rospy.init_node('stepper_motor', anonymous=True)

    stepper = stepper_class.Stepper_Control(ch0, ch1)
    stepper.move_motor()

  except rospy.ROSInterruptException:
    pass


