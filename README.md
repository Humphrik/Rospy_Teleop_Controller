To run the code:
* roscore
* roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
* roslaunch turtlebot3_manipulation_moveit_config move_group.launch
* rosrun Rospy_Teleop_Controller drive_system.py

### Project Description
The goal of the project was to create controller teleoperation, in which connecting a controller would give a user full funtionality 
of that robot's wheels and arm through controller manipulation. This is interesting because it makes moving the robot a lot easier, 
especially the arm, and it contributes a physical, hardware component that was missing in this class. Through this project, we were able to 
make the robot move, turn, and grab things purely through buttons and joysticks on a controller. The three main components of this project 
are controller input, navigation, and inverse kinematics. The controller input controls the navigation and implements inverse kinematics in 
order to move the arm the way the user wants it to move. 

### System Architecture
The code consists of running one class object that consists of callback functions corresponding to the buttons that have any influence 
over the navigation and kinematics of the robot. This is possible through pygame and pygame-controller, which are imported at the top of 
drive_system.py. The controller input component is made possible through pygame-controller. An example on how to use it is in the other py 
file. The run function in the class Robot_Controller is what onnects the callback functions we've made to the buttons on the controller. 
Navigation is carried out through the two callback functions in the class that connect to the two joysticks, publishing velocities based on 
the values being published by the joysticks. The inverse kinematics part is implemented through the other callback functions that connect 
to the first pair of shoulder buttons and hat. The values from the controller are taken in and used in the helper function called 
update_arm, where inverse kinematics and gripper manipulation are implemented.

### Challenges
With such an unusual topic, we ran the risk of not being able to find the right support necessary to implement this project. Luckily, we 
found all the right things to use in order to make the connection from controller to robot. Another problem was inverse kinematics as a 
whole. Attempting 3 DOF inverse kinematics proved to be quite a challenge. In the end, it came out as 2 DOF inverse kinematics, with the 
gripper adjusting with the motion of the arm to maintain a suitable angle. Either way, we realized the user should have more control over 
the angle of the gripper, so we produced functionality for that with the second pair of shoulder functions. Another challenge was taking 
into account the possibility of the robot tipping over since the user has so much freedom with arm movement. That is still a problem not 
accounted for.

### Future Work
Possible enhancements of this project would include full implementation of 3 DOF inverse kinematics, while still including the gripper 
adjustment option we have. Along with arm motion, we would prevent tipping of the robot by preventing the arm from going past certain points 
with specific orientations that would move the center of mass too far away from the robot. Another concept we talked about was having 
preset actions that the user may find useful for certain, common tasks. 

### Takeaways
* There are a lot of resources online for many ideas that may seem complicated or inapplicable at first. Possibilities are endless.
* Thinking about how the user would interact with the robot was useful in deciding what to apply and for which buttons. Human-computer interaction was a very important factor to consider.
* Inverse kinematics should never be underestimated.
* Using a controller to move the robot was way easier and should be used for everything from now on (maybe you can use it for your next class).

### Gifs of behavior
- Here is the robot being driven with 2 joysticks


![Drive System](Teleop_Drive.gif)


- A Demonstration of the freedom of movement for the arm.


![motion](Arm_Movement.gif)





