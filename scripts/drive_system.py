#!/usr/bin/env python3
""" This script publishes ROS messages containing the 3D coordinates of a single point """
import rospy
import pygame
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
        # print ("Left js value changed to ", UDval)
        self.cmd_pub.publish(Vector3(self.robot_vel, 0, 0), Vector3(0, 0, self.robot_turn))

    def onRightStickChange(self, LRval, UDval):
        self.robot_turn = -LRval
        # print ("Right js value changed to ", LRval)
        self.cmd_pub.publish(Vector3(self.robot_vel, 0, 0), Vector3(0, 0, self.robot_turn))




    def __init__(self):
        rospy.init_node('send_robot_action')    # initialize our ROS node
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)


        # Initialze the robot parameters we want to track.
        self.robot_vel = 0 #[-1,1]
        self.robot_turn = 0 #[-1,1]
                



    def run(self):
        print ("Controller is initializing...")
        try:
            cnt = RobotController("Robot controller program", self.init_status,
                    # Joystick drive
                    leftStickChanged  = self.onLeftStickChange,
                    rightStickChanged = self.onRightStickChange)
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
