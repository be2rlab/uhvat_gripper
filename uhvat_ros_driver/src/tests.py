#!/usr/bin/env python3
import rospy
import time
from uhvat_festolike_driver.srv import SetGripperState


def main(state):
    rospy.wait_for_service('gripper_state')
    try:
        set_gripper_state = rospy.ServiceProxy('gripper_state', SetGripperState)
        set_gripper_state(state)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    main(0)
    time.sleep(5)

    main(1)
    time.sleep(1)

    main(2)
    time.sleep(1)

    main(3)
    time.sleep(1)

    main(1)
    time.sleep(2)
    main(4)
    time.sleep(3)

    main(1)
    time.sleep(2)
    main(6)
    time.sleep(3)

    main(1)
    time.sleep(2)
    main(6)
    time.sleep(3)

    main(1)
    time.sleep(2)
    main(6)
    time.sleep(3)


    main(0)
    time.sleep(1)
