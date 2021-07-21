#!/usr/bin/env python3
import os, time
import rospy
import math
from dynamixel_sdk import *
from uhvat_festolike_driver.srv import *

from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

import threading
lock = threading.Lock()

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# gripper setup
LEFT_DXL_ID             = 13
RIGHT_DXL_ID            = 14


LEFT_CLOSED_POSITION = 512
RIGHT_CLOSED_POSITION = 512
FULL_CLOSED = 80

LEFT_OPENED_POSITION = 512 - 300
RIGHT_OPENED_POSITION = 515 + 300
LESS_WIDE_OPENED = 200


# connection setup
ADDR_COMPLIANCE_CW = 28
ADDR_COMPLIANCE_CC = 29

ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
ADDR_PRESENT_POSITION   = 36
PROTOCOL_VERSION        = 1.0               # See which protocol version is used in the Dynamixel

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


class GripperDriver(object):

    def __init__(self) -> None:
        super().__init__()
        rospy.init_node('driver_node')
        rospy.Service('gripper_state', SetGripperState, self.__set_gripper_state)
        rospy.Service('gripper_reboot', Empty, self.__reboot)
        # rospy.Service('gripper_change_mode', Empty, self.__change_mode)
        
        # state publisher
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.mode = 'position'
        self.left_pos, self.right_pos = 0, 0
        self.prev_left_pos, self.prev_right_pos = 0, 0
        self.driver_state = 'ready'  # `read` or `write` or `ready`

    def init(self):
        """
        connect to usb
        motors turn on
        """
        try:
            portHandler.openPort()
            portHandler.setBaudRate(BAUDRATE)
            print("Usb connection fine")

            self.__position_mode()  
            # self.__speed_mode()
        except Exception as e:
            print(e)
            print("Gripper connection failed")
            print("Press any key to terminate...")
            getch()
            quit()
        print("Gripper driver is working")

    def __position_mode(self):
        for id in [LEFT_DXL_ID, RIGHT_DXL_ID]:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 32, 0)

            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 6, 0)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 8, 1023)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 34, 1023)

            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1)
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 25, 1)

            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                print("Press any key to terminate...")
                getch()
                quit()
            else:
                print("Servomotors has been successfully connected")
            
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_COMPLIANCE_CW, 254)
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_COMPLIANCE_CC, 254)

    def __speed_mode(self):
        # Enable Dynamixel Torque
        for id in [LEFT_DXL_ID, RIGHT_DXL_ID]:                        
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 6, 0)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 8, 0)
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1)
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 25, 1)

            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                print("Press any key to terminate...")
                getch()
                quit()
            else:
                print("Servomotors has been successfully connected")


    def __get_gripper_state(self):
        l_dxl_present_position, l_dxl_comm_result, l_dxl_error = packetHandler.read4ByteTxRx(portHandler, LEFT_DXL_ID, ADDR_PRESENT_POSITION)
        r_dxl_present_position, r_dxl_comm_result, r_dxl_error = packetHandler.read4ByteTxRx(portHandler, RIGHT_DXL_ID, ADDR_PRESENT_POSITION)
        return l_dxl_present_position, r_dxl_present_position

    def __set_gripper_state(self, req):
        lock.acquire()
        if self.driver_state == 'ready':
            self.driver_state = 'write'
            while portHandler.is_using:
                time.sleep(0.01)

            if req.state == 0:   # wide opened
                self.__switch('position')
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, LEFT_DXL_ID, ADDR_GOAL_POSITION, LEFT_OPENED_POSITION)
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, RIGHT_DXL_ID, ADDR_GOAL_POSITION, RIGHT_OPENED_POSITION)
            elif req.state == 1: # less wide opened
                self.__switch('position')
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, LEFT_DXL_ID, ADDR_GOAL_POSITION, LEFT_OPENED_POSITION + LESS_WIDE_OPENED)
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, RIGHT_DXL_ID, ADDR_GOAL_POSITION, RIGHT_OPENED_POSITION - LESS_WIDE_OPENED)
            elif req.state == 2: # before position of full closed
                self.__switch('position')
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, LEFT_DXL_ID, ADDR_GOAL_POSITION, LEFT_CLOSED_POSITION)
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, RIGHT_DXL_ID, ADDR_GOAL_POSITION, RIGHT_CLOSED_POSITION)
            elif req.state == 3: # full closed
                self.__switch('position')
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, LEFT_DXL_ID, ADDR_GOAL_POSITION, LEFT_CLOSED_POSITION + FULL_CLOSED)
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, RIGHT_DXL_ID, ADDR_GOAL_POSITION, RIGHT_CLOSED_POSITION - FULL_CLOSED)
            elif req.state == 4: # speed -- low force
                self.__switch('speed')
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, LEFT_DXL_ID, 32, 150)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, RIGHT_DXL_ID, 32, 1023 + 150)
            elif req.state == 5: # speed -- more force
                self.__switch('speed')
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, LEFT_DXL_ID, 32, 300)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, RIGHT_DXL_ID, 32, 1023 + 300)
            elif req.state == 6: # speed -- max force
                self.__switch('speed')
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, LEFT_DXL_ID, 32, 450)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, RIGHT_DXL_ID, 32, 1023 + 450)
            else:
                rospy.logwarn(f"There is no state \"{req.state}\"")
            
            self.driver_state = 'ready'
        lock.release()
        return []

    def __reboot(self, req):
        for id in [LEFT_DXL_ID, RIGHT_DXL_ID]:                        
            dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, id)
        self.__position_mode()
        rospy.logwarn('Rebooted!')

    def __switch(self, mode):
        self.mode = mode
        if mode == 'position':
            self.__position_mode()
        else:
            self.__speed_mode()
        rospy.loginfo(f"mode: {self.mode}")

    def __change_mode(self, req):
        if self.mode == 'position':
            self.__switch('speed')
        else:
            self.__switch('position')


    def spin(self):

        """ 
            The gripper illustration:
            
              | |
              |.|         --- flange
            \|   |/       --- fingers
            0     1       --- indexes in urdf
         left    right    --- view from top direction

            Opened (full) position is [tics]:
        214 810 --- let's it is a zero position
            Closed (full) position is [tics]:
        572 438 --- it is a limit position

        [tics to deg] => 
            alpha = (pos - zero_position) / 1024
        
        
        """
        LEFT_ZERO_POSITION = 214 * 300 / 1024
        RIGHT_ZERO_POSITION = 810 * 300 / 1024

        rate = rospy.Rate(300)
        while not rospy.is_shutdown():

            if self.driver_state == 'ready':
                self.driver_state == 'read'
                if not portHandler.is_using:
                    l, r = self.__get_gripper_state()
                    if l + r < 3000:
                        self.left_pos, self.right_pos = l, r
                    # print(self.left_pos, self.right_pos)
                self.driver_state == 'ready'

            # transform [tics] to [rad] with offset
            left_pos_rad = (LEFT_ZERO_POSITION - self.left_pos * 300 / 1024) * math.pi / 180
            right_pos_rad = (RIGHT_ZERO_POSITION - self.right_pos * 300 / 1024) * math.pi / 180

            # filtring measurements
            left_pos_rad = (left_pos_rad + self.prev_left_pos) / 2
            right_pos_rad = (right_pos_rad + self.prev_right_pos) / 2
            self.prev_left_pos, self.prev_right_pos = left_pos_rad, right_pos_rad

            # print(left_pos_rad, right_pos_rad)    # debug msg

            try:
                msg = JointState()
                msg.header.stamp.secs = rospy.get_rostime().secs
                msg.header.stamp.nsecs = rospy.get_rostime().nsecs
                
                # TODO using lists below
                msg.name.append('uhvat_joint_1')    # left but right on the arm :)
                msg.name.append('uhvat_joint_2')
                msg.position.append(left_pos_rad)  # left
                msg.position.append(right_pos_rad)

                self.pub.publish(msg)
            except Exception as e:
                print(e)

            rate.sleep()



def main():

    driver = GripperDriver()
    driver.init()
    driver.spin()


if __name__ == '__main__':
    main()
