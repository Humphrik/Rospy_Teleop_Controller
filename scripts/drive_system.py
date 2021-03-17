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
        if (abs(self.robot_vel) <= 0.06):
            self.robot_vel = 0
        if (self.robot_safety and self.obstruction):
            if (self.obstruction < 0 and self.robot_vel < 0):
                self.robot_vel = 0
            elif (self.obstruction > 0 and self.robot_vel > 0):
                self.robot_vel = 0
            else:
                pass
        # print ("Left js value changed to ", UDval)
        self.cmd_pub.publish(Vector3(self.robot_vel, 0, 0), Vector3(0, 0, self.robot_turn))

    def onRightStickChange(self, LRval, UDval):
        self.robot_turn = -LRval
        if (abs(self.robot_turn) <= 0.06):
            self.robot_turn = 0
        # print ("Right js value changed to ", LRval)
        self.cmd_pub.publish(Vector3(self.robot_vel, 0, 0), Vector3(0, 0, self.robot_turn))

    def onRightBtn1Change(self, val):
        self.z_dir = val
        
    def onLeftBtn1Change(self, val):
        self.z_dir = -val

    def onRightBtn2Change(self, val):
        if (val > 0):
            self.grip_ldir = val
        else:
            self.grip_ldir = 0

    def onLeftBtn2Change(self, val):
        if (val > 0 and self.grip_ldir == 0):
            self.grip_rdir = -val
        else:
            self.grip_rdir = 0

    def onHatChange(self, LRval, UDval):
        # Setting the directions.
        self.s_dir = UDval
        self.phi_dir = LRval

    def onTriangleBtnChange(self, val):
        self.robot_gripper_open = val

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
        self.data = LaserScan()
        rospy.Subscriber("scan", LaserScan, self.robot_scan_received)

        # parameter for LIDAR scan to detect obstruction
        self.obstruction = 0

        # Initialze the robot parameters we want to track.
        self.robot_vel = 0 #[-1,1]
        self.robot_turn = 0 #[-1,1]

        # The arm lengths, maybe.
        self.l1 = 0.12
        self.l2 = 0.12

        self.robot_arm_curr_pos = [self.l2, 0, self.l1] # (s, phi, z). Corresponds to (x, joint1, 1 in lecture calculations).
        self.robot_group_arm_curr = [0, 0, 0, 0] # in radians
        self.robot_group_gripper_curr = [0.0, 0.0] # we can open when pressing down on TRIANGLE (?), closes when not
        self.robot_safety = 0 # turns into 1 when SQUARE is pressed once? Back to 0 when pressed again? (With printed message of change

        # Parameters for moving arm.
        self.s_dir = 0
        self.phi_dir = 0
        self.z_dir = 0

        # Parameters for moving gripper
        self.robot_gripper_open = 0 # takes value of button that triggers gripper movement
        self.robot_grip = 1 # multiplies to self.robot_gripper_open to change direction after reaching a limit
        

        self.grip_ldir = 0 # The angle of the wrist joint.
        self.grip_rdir = 0
        

        self.grip_ang_dir = 1 # either 1 or -1 to chane direction of motion after reaching certain limits
        self.wrist_offset = 0
        

        # initialize robot arm position
        self.move_group_gripper.go(self.robot_group_gripper_curr, wait=True)
        self.move_group_arm.go(self.robot_group_arm_curr, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.stop()

        """
        # Math test.
        xy = self.get_xy(0, 0)
        print (xy)
        qs = self.get_angles(xy[0], xy[1])
        print (qs)
        """


    def robot_scan_received(self, data):
        if (self.robot_safety):
            if (data.ranges[0] < 0.7):
                if (self.robot_vel > 0):
                    self.obstruction = 1
                    print("CAUTION: Obstruction ahead")
            elif (data.ranges[180] < 0.7):
                if (self.robot_vel < 0):
                    self.obstruction = -1
                    print("CAUTION: Obstruction ahead")
            else:
                self.obstruction = 0
        else:
            self.obstruction = 0
            if (data.ranges[0] < 0.7):
                if (self.robot_vel > 0):
                    print("CAUTION: Obstruction ahead")
            elif (data.ranges[180] < 0.7):
                if (self.robot_vel < 0):
                    print("CAUTION: Obstruction ahead")


    def update_arm(self):
        # Constants to adjust the speed of the robot.
        ds = 0.005
        dphi = math.pi/90
        dz = 0.005
        d4 = 0.1
        dg = 0.001

        # Get the current wrist positions.
        self.robot_group_arm_curr = self.move_group_arm.get_current_joint_values()


        if (self.s_dir == 0 and self.phi_dir == 0 and self.z_dir == 0 and self.grip_ldir == 0 and self.grip_rdir == 0):
            self.move_group_arm.stop()
            self.wrist_offset = self.robot_group_arm_curr[3] + (self.robot_group_arm_curr[1] + self.robot_group_arm_curr[2])
            return


        # Turning the arm CW/CCW. Phi and joint 1's values are the same.
        if (self.phi_dir != 0):
            self.robot_group_arm_curr[0] += dphi * -self.phi_dir
            if (abs(self.robot_group_arm_curr[0]) > 2.86):
                self.robot_group_arm_curr[0] = 2.86 * -self.phi_dir
            self.robot_arm_curr_pos[1] = self.robot_group_arm_curr[0] 

        if (self.s_dir != 0 or self.z_dir != 0):
            pos = self.get_xy(self.robot_group_arm_curr[1], self.robot_group_arm_curr[2])
            pos[0] += ds * self.s_dir
            pos[1] += dz * self.z_dir
 
            qs = self.get_angles(pos[0], pos[1])
            print ("angles: ", qs)
            if(not math.isnan(qs[0])):
                self.robot_group_arm_curr[1] = qs[0]
                self.robot_group_arm_curr[2] = qs[1]
            else:
                print ("Location out of bounds!")


        
        #self.robot_group_arm_curr[3] = self.wrist_offset + (math.pi/2 - self.robot_group_arm_curr[1]) - (math.pi/2 + self.robot_group_arm_curr[2])
        self.robot_group_arm_curr[3] = self.wrist_offset -(self.robot_group_arm_curr[1] + self.robot_group_arm_curr[2])

        # change gripper angle by user
        if (self.grip_ldir > 0):
            if (self.robot_group_arm_curr[3] + self.wrist_offset < 2.2):
                self.wrist_offset += d4
        elif (self.grip_rdir < 0):
            if (self.robot_group_arm_curr[3] + self.wrist_offset > -2.2):
                self.wrist_offset -= d4
        else:
             self.wrist_offset = self.robot_group_arm_curr[3] + (self.robot_group_arm_curr[1] + self.robot_group_arm_curr[2])




        # move gripper when called on, prepetually opening and closing
        if (self.robot_gripper_open):
            if (self.robot_group_gripper_curr[0] > 0.008):
                self.robot_grip = -1
            elif (self.robot_group_gripper_curr[0] < 0.001):
                self.robot_grip = 1
            self.robot_group_gripper_curr[0] += dg * self.robot_gripper_open * self.robot_grip
            self.robot_group_gripper_curr[1] += dg * self.robot_gripper_open * self.robot_grip
            print("Gripper opening: ", self.robot_group_gripper_curr[0])


        # Adjust the motion of the arm.
        try:
             self.move_group_arm.go(self.robot_group_arm_curr, wait=False)
        except(moveit_commander.exception.MoveItCommanderException):
             print ("Desired motion out of bounds!")
             self.move_group_arm.stop()


    def get_angles(self, x, y):
        l1 = self.l1
        l2 = self.l2
        temp2  = ((l1**2) + (l2**2) - (x**2) - (y**2)) / (2*l1*l2)
        if (abs(temp2) > 1):
            print ("Point not reachable!")
            return [float("NaN"), float("NaN")]
        q2 = -(math.pi -math.acos(temp2))
        temp1_l = l2 * math.sin(q2)
        temp1_r = l1 + l2 * math.cos(q2)
        q1 = math.atan(y/x) - math.atan2(temp1_l, temp1_r)

        # Again, I'm applying the special transormations from problem contexxt to joint angles.
        return [math.pi/2 - q1, -(math.pi/2 + q2)]




    def get_xy (self, q1, q2):
        # Note the tranformations on q1 and q2 from the joint angles to the context of this problem.
        q1prime = math.pi/2 - q1
        q2prime = -(math.pi/2 + q2)
        x = self.l1 * math.cos(q1prime) + self.l2 * math.cos(q1prime + q2prime)
        y = self.l1 * math.sin(q1prime) + self.l2 * math.sin(q1prime + q2prime)
        return [x, y]


    def run(self):
        print ("Controller is initializing...")
        try:
            cnt = RobotController("Robot controller program", self.init_status,
                    # Joystick drive
                    leftStickChanged  = self.onLeftStickChange,
                    rightStickChanged = self.onRightStickChange,
                    rightBtn1Changed = self.onRightBtn1Change,
                    leftBtn1Changed = self.onLeftBtn1Change,
                    rightTriggerChanged = self.onRightBtn2Change,
                    leftTriggerChanged = self.onLeftBtn2Change,
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
