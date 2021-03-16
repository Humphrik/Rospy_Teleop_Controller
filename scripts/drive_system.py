#!/usr/bin/env python3
""" This script publishes ROS messages containing the 3D coordinates of a single point """
import rospy
import math
import pygame
import moveit_commander
import numpy as np
from sensor_msgs.msg import LaserScan
from pygamecontroller import RobotController
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Robot_Controller:


    def init_status(self, status):
        if (status == 0):
            print ("Controller connected!")
        elif (status < 0):
            print ("No supported controller found!")


    # CALLBACK FUNCTIONS GO HERE!
    
    def onLeftStickChange(self, LRval, UDval):
        self.robot_vel = -UDval
        if (abs(self.robot_vel) <= 0.05):
            self.robot_vel = 0
        # print ("Left js value changed to ", UDval)
        self.cmd_pub.publish(Vector3(self.robot_vel, 0, 0), Vector3(0, 0, self.robot_turn))

    def onRightStickChange(self, LRval, UDval):
        self.robot_turn = -LRval
        if (abs(self.robot_turn) <= 0.05):
            self.robot_turn = 0
        # print ("Right js value changed to ", LRval)
        self.cmd_pub.publish(Vector3(self.robot_vel, 0, 0), Vector3(0, 0, self.robot_turn))

    def onRightBtn2Change(self, val):
        if (val > 0):
            self.robot_group_arm_curr[0] += -val
            self.move_group_arm.go(self.robot_group_arm, wait=True)
            self.move_group_arm.stop()
        
    def onLeftBtn2Change(self, val):
        if (val > 0):
            self.robot_group_arm_curr[0] += val
            self.move_group_arm.go(self.robot_group_arm, wait=True)
            self.move_group_arm.stop()

    def onHatChange(self, LRval, UDval):
        # L1 ~ 130, L2 ~ 130 mm
        L = 130
        denom = 2*L*L
        x = self.robot_arm_curr_pos[1] + LRval
        y = self.robot_arm_curr_pos[2] + UDval
        d = x**2 + y**2
        numer = denom - d
        new_q2_1 = np.pi - math.acos(numer/denom)
        new_q2_2 = math.acos(-numer/denom)
        new_q1_1 = math.atan(y/x) - math.atan((L*math.sin(new_q2_1))/(L + L*math.cos(new_q2_1)))
        new_q1_2 = math.atan(y/x) - math.atan((L*math.sin(new_q2_2))/(L + L*math.cos(new_q2_2)))
        #print(x, y, '\n', self.robot_arm_curr_pos[3], new_q2_1, new_q2_2, '\n', self.robot_arm_curr_pos[4], new_q1_1, new_q1_2)
        self.robot_arm_curr_pos = [d, x, y, new_q1_1, new_q2_1]
        self.robot_group_arm_curr[1] = new_q1_1
        self.robot_group_arm_curr[2] = math.radians(90) - new_q2_1
        self.move_group_arm.go(self.robot_group_arm_curr, wait=True)
        self.move_group_arm.stop()
        #TODO
        pass

    def onTriangleBtnChange(self, val):
        if (val):
            self.robot_group_gripper_curr = [0.009, 0.009]
        else:
            self.robot_group_gripper_curr = [0.0, 0.0]
        self.move_group_gripper.go(self.robot_group_gripper_curr, wait=True)
        self.move_group_gripper.stop()

    def onSquareBtnChange(self, val):
        if (self.robot_safety and val):
            self.robot_safety = 0
            print("Safety mode is OFF")
        elif (val):
            self.robot_safety = 1
            print("Safety mode is ON")
        else:
            pass


    def __init__(self):
        rospy.init_node('send_robot_action')    # initialize our ROS node
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # subscribing to LIDAR data scan for safety mode
#        self.data = LaserScan()
 #       rospy.Subscriber("scan", LaserScan, self.robot_scan_received)

        # Initialze the robot parameters we want to track.
        self.robot_vel = 0 #[-1,1]
        self.robot_turn = 0 #[-1,1]
        self.robot_arm_curr_pos = [183.85, 130, 130, math.radians(0), math.radians(90)] # [d, x, y, q1, q2] d, x, y are in mm
        self.robot_group_arm_curr = [0, 0, 0, 0] # in radians
        #TODO functions for parameters below
        self.robot_group_gripper_curr = [0.0, 0.0] # we can open when pressing down on TRIANGLE (?), closes when not
        self.robot_safety = 0 # turns into 1 when SQUARE is pressed once? Back to 0 when pressed again? (With printed message of change
        # initialize robot arm position
        self.move_group_gripper.go(self.robot_group_gripper_curr, wait=True)
        self.move_group_arm.go(self.robot_group_arm_curr, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()

        rospy.sleep(3)


#    def robot_scan_received(self, data):
 #       if (self.robot_safety):
  #          if (data.ranges[0] < (self.robot_group_arm_curr[0]+0.1)):
   #             if (self.robot_vel > 0):
    #                self.cmd_pub.publish(Vector3(0, 0, 0), Vector3(0, 0, self.robot_turn))
     #               print("CAUTION: Obstruction ahead")
      #      elif (data.ranges[180] < (self.robot_group_arm_curr[0]+0.1)):
       #         if (self.robot_vel > 0):
        #            self.cmd_pub.publish(Vector3(0, 0, 0), Vector3(0, 0, self.robot_turn))
         #           print("CAUTION: Obstruction ahead")


    def run(self):
        print ("Controller is initializing...")
        try:
            cnt = RobotController("Robot controller program", self.init_status,
                    # Joystick drive
                    leftStickChanged  = self.onLeftStickChange,
                    rightStickChanged = self.onRightStickChange,
                    rightBtn2Changed = self.onRightBtn2Change,
                    leftBtn2Changed = self.onLeftBtn2Change,
                    hatChanged = self.onHatChange,
                    triangleBtnChanged = self.onTriangleBtnChange,
                    squareBtnChanged = self.onSquareBtnChange)
                    # PUT MORE BUTTONS HERE!

            # Controller initialzed!
            if cnt.initialised :
                keepRunning = True
            else:
                keepRunning = False


            # THE MAIN LOOP.
            r = rospy.Rate(10)
            while keepRunning and not rospy.is_shutdown():
                # Put stuff here while controller is running.
                keepRunning = cnt.controllerStatus()
                r.sleep()

        finally:
            pygame.quit()
            print ("Quitting...")

            

if (__name__ == '__main__'):
    ROS = Robot_Controller()
    ROS.run();
