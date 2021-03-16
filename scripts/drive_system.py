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

    def onRightBtn1Change(self, val):
        self.z_dir = val
        
        
    def onLeftBtn1Change(self, val):
        self.z_dir = -val

    def onHatChange(self, LRval, UDval):
        # Setting the directions.
        self.s_dir = UDval
        self.phi_dir = LRval


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
        self.robot_arm_curr_pos = [0, 0, 0] # (s, phi, z). Corresponds to (x, joint1, 1 in lecture calculations).
        self.robot_group_arm_curr = [0, 0, 0, 0] # in radians
        self.robot_group_gripper_curr = [0.0, 0.0] # we can open when pressing down on TRIANGLE (?), closes when not
        self.robot_safety = 0 # turns into 1 when SQUARE is pressed once? Back to 0 when pressed again? (With printed message of change

        # Parameters for moving arm.
        self.s_dir = 0
        self.phi_dir = 0
        self.z_dir = 0


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


    def update_arm(self):
        # The lengths of the arms (maybe)
        l1 = 0.208
        l2 = 0.068
        # Constants to adjust the speed of the robot.
        ds = 0.05
        dphi = math.pi/90
        dz = 0.05

        if (self.s_dir == 0 and self.phi_dir == 0 and self.z_dir == 0):
            self.move_group_arm.stop()
            return

        # Update the current positions.
        self.robot_group_arm_curr = self.move_group_arm.get_current_joint_values()
        print(self.robot_group_arm_curr)

        # Turning the arm CW/CCW. Phi and joint 1's values are the same.
        if (self.phi_dir != 0):
            self.robot_group_arm_curr[0] += dphi * -self.phi_dir
            if (abs(self.robot_group_arm_curr[0])  > 2.86):
                self.robot_group_arm_curr[0] = 2.86 * -self.phi_dir
            self.robot_arm_curr_pos[1] = self.robot_group_arm_curr[0] 

        # TODO: Adjust joints 2 and 3 based on the change in s and z (x and y).

        # TODO: Adjust joint four to always be the sum of joint 2 and 3.
        # Note that we can also program an offset for this joint to be changed via buttons.



        # Adjust the motion of the arm.
        try:
             self.move_group_arm.go(self.robot_group_arm_curr, wait=False)
        except(moveit_commander.exception.MoveItCommanderException):
             print ("Desired motion out of bounds!")
             self.move_group_arm.stop()




    def run(self):
        print ("Controller is initializing...")
        try:
            cnt = RobotController("Robot controller program", self.init_status,
                    # Joystick drive
                    leftStickChanged  = self.onLeftStickChange,
                    rightStickChanged = self.onRightStickChange,
                    rightBtn1Changed = self.onRightBtn1Change,
                    leftBtn1Changed = self.onLeftBtn1Change,
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
                self.update_arm()
                r.sleep()

        finally:
            pygame.quit()
            print ("Quitting...")

            

if (__name__ == '__main__'):
    ROS = Robot_Controller()
    ROS.run();
