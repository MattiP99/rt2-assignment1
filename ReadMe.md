Author: Mattia Piras

#First assignment for Research Track 2: Course of Robotics Engineering Unige  
---------------------------------------------------------------------------------------------------------

In this project, a robot moves in an environment initially unknown to it. Thanks to laser scanners mounted on board the robot is able to build a map of the environment and move without encountering obstacles. It is required that the robot can move in 3 different modes:

    - setting the coordinates of a goal to be achieved and the robot would have to dynamically calculate a path to reach it
    - controlling it via keyboard
    - controlling it via keyboard with assistance to avoid obstacles
    
##INSTALLING AND RUNNING
---------------------------------------------------------------------------------------------------------
##Installing and running

Here's some useful informations regarding how to the nodes and the simulation environment. First of all, xterm, a standard terminal emulator, is needed. You can install xterm by entering the following commands in the terminal:

	sudo apt update
	sudo apt-get install xterm

I created a launch file in the launch directory that executes all the necessary nodes and the simulation environment. You can run the programs by entering the following command:

	roslaunch final_assignment launchAll.launch 

The three modalities and the UI nodes will run on different xterm terminals. You can control the robot with the three different modalities by reerring to each xterm terminal that will appear after the launch file execution.

If any of the three node terminates, the launch file will terminates all the nodes.



##Environment and Mapping
-----------------------------------------------------------------------------------------------------

The robot moves in the environment in the figure (Gazebo view):

<img width="782" alt="gazebo" src="https://user-images.githubusercontent.com/92534255/155940743-e88f5ab1-e097-4025-a3a5-460ad08b761e.png">


Initially the robot does not have a map of the surrounding environment, but can build it thanks to the laser scanner it is equipped with and thanks to the gmapping package. 

The final map, visible on Rviz is as follows:

<img width="487" alt="rviz" src="https://user-images.githubusercontent.com/92534255/155940790-33f4e580-fc23-494b-9d37-f9bdb1d97679.png">



##Project structure
------------------------------------------------------------------------------------------------------

##User Interface Node

The User Interface node handles the user keyboard inputs. Here's a legend of the allowed commands:

    '1' keyboard key is used for choosing the autonomously reaching modality;
    '2' keyboard key is used for the free keyboard driving modality;
    '3' keyboard key is used for the free keyboard driving modality with a collision avoidance algorithm;
    '4' keyboard key is used for quitting the application and terminates all nodes; This node is very simply designed. Essentially, a function called interpreter() is looped inside a while not rospy.is_shutdown(): loop. This function gets the keyboard user input and changes the ROS parameter active depending on which modality was chosen. Here's the interpreter() code:

def interpreter():
	#Function that receives inputs and sets all the ROS parameters
	
 	command = input(bcolors.HEADER + 'Choose a modality: \n' + bcolors.ENDC) # Stores the input key
	
	if command == "0":
		rospy.set_param('active', 0) # if the active parameter is 0 the current goal is canceled
		print(bcolors.OKGREEN + "No modality is active, please choose one for controlling the robot" + bcolors.ENDC) # Sysytem in idle state
		active_=rospy.get_param("/active")
	elif command == "1": # Modality one chosen
		rospy.set_param('active', 0) # Useful for changing goal
		active_=rospy.get_param("/active")
		# Receive the desired cooridnates as input
		des_x_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired x position: " + bcolors.ENDC))
		des_y_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired y position: " + bcolors.ENDC))
		rospy.set_param('des_pos_x', des_x_input) # Update the desired X coordinate
		rospy.set_param('des_pos_y', des_y_input) # Update the desired Y coordinate
		rospy.set_param('active', 1) # Modality 1 active
	elif command == "2": # Modality two chosen
		rospy.set_param('active', 2) # Modality two active
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 2 is active." + bcolors.ENDC)
		print(bcolors.BOLD + bcolors.HEADER + "Use the 'my_teleop_twist_keyboard' xterm terminal to control the robot" + bcolors.ENDC)
		active_=rospy.get_param("/active")
	elif command == "3": # Modality three chosen
		rospy.set_param('active', 3) # # Modality three active
		active_=rospy.get_param("/active")
	elif command == "4": # Exit command
		print(bcolors.WARNING + bcolors.BOLD + "Exiting..." + bcolors.ENDC)
		os.kill(os.getpid(), signal.SIGKILL) # Kill the current process
	else:
		print(bcolors.FAIL + bcolors.BOLD + "Wrong key! Use the shown commands " + bcolors.ENDC)

###Autonomously reaching node (First Modality)

This node makes the robot autonomously reach a x,y position inserted by the user. The robot can reach the user's x,y coordinates thanks to the 'move_base' action server. The robot is going to plan the path through the Dijkstra's algorithm. When the first modality is selected, the UI.py node sets the active ROS parameter to 1 letting the first modality's loop to run all the necessary code for sending the desired goal. The desired x, y coordinates are ROS parameters too and they are set by the UI.py node.

When the first modality is running and a goal is received, this node uses the send_goal(goal_msg, done_cb(), active_cb(), feedback_cb()) function for asking the action server to compute the path planning for the desired goal.

    - goal_msg is a MoveBaseGoal() action message containing all the information about the desired goal (i.e. x, y coordinates, referencing frame, etc... )
    
    - done_cb(status, result) is a callback function called after the execution of the action server. It gives the client information about the termination of the 
    		goal process. In particular, this callback function returns a value that is stored in am int variable that I called status. Depending on the value of 		      this variable the client knows the status of the goal processing.
    - active_cb() is a callback funtion called before the execution of the action server. I used this callback funtion in order to take into account the number of 			processed goals.
    
    - feedback_cb(feedback) is a callback funtion called durning the execution of the action server. It returns feedbacks about the goal processing.



thanks to the done_cb(status, result) function I could manage the result of the goal achievement. Here's the done_cb(status, result) code:

def done_cb(status, result):
	# Function called after goal was processed. It is used to notify the client on the current status of every goal in the system.
	global client
	global achieved
	global goal_cont
	goal_cont += 1 # Increment goal counter
	if status == 2:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing. Execution terminated." + bcolors.ENDC)
		cont = 1
		return
	if status == 3:
		print(bcolors.OKGREEN + bcolors.UNDERLINE + bcolors.BOLD + "Goal successfully achieved" + bcolors.ENDC)
		cont = 1
		achieved = True
		return
	if status == 4:
		print(bcolors.FAIL + "Timeout expired, the desired poition is not reachable. Goal aborted." + bcolors.ENDC)
		cont = 1
		return
	if status == 5:
		print(bcolors.FAIL + "The goal was rejected" + bcolors.ENDC)
		cont = 1
		return
	if status == 6:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing and has not yet completed execution"+ bcolors.ENDC)
		cont = 1
		return
	if status == 8:
		print(bcolors.FAIL + "The goal received a cancel request before it started executing and was successfully cancelled."+ bcolors.ENDC)
		cont = 1
		return

A concept that I want to point out is the that the achievement of the goal implies a cancel of such goal. So, I had to use a flag (bool achieved) to differentiate the case in which a goal was achieved (in this case the cacel request of an already canceled goal may cause an error, so I avoided to send the cancel request to the action server) and the case in which the user decides to send a cancel request before the goal achievement (in this case we must send a cancel request to the server).

This is a piece of code about the differentiation that I just pointed out:

if flag == 0 and achieved == False: # If we are in Idle state but a goal was not achieved we need to cancel the goal
		print("Modality 1 is currently in idle state\n")
		client.cancel_goal() # Send a cancel request
		flag = 1 # Ready to set a new goal if this modality is unlocked
if achieved == True: # If a goal was achieved there's no need to cancel the goal
		flag = 1
		achieved = False

###Free drive with keyboard node (Second Modality)

This node reads inputs from the keyboar and publishes a Twist() message to the cmd_vel topic. Basically I relied on the teleop_twist_keyboard code. So, the functionality is the same:

#### Instructions
Moving around:
				u i o
				j k l
				m , .
For Holonomic mode (strafing), hold down the shift key:
---------------------------

				U I O
				J K L
				M < >
							
t : up (+z)
b : down (-z)
anything else : stop

q/z : increase/decrease max speeds by 10%

w/x : increase/decrease only linear speed by 10%

e/c : increase/decrease only angular speed by 10%

I modified some lines of code in order to adapt the process on my needs. This modality is a total free driving algorithm, that means that the robot can obviously bump into the obstacles. The main changes I've made from the teleop_twist_keyboard node are:

    Insertion of an if(active == 2) statement in order to block the code when another modality is running.
    The keys now must be kept pressed in order to move the robot. I did this by setting the key_timeout variable to 0.1. Such variable was the select() timeout. That means that the select() function waits 0.1 seconds for new inputs at every loop.

###Free drive with keyboard and collision avoidance algorithm node (Third Modality)

This node reads inputs from the keyboard and publishes a Twist() message to the /cmd_vel topic. Basically I relied on the teleop_twist_keyboard code and therefore the functionality is quite the same:

#### Instructions

Reading from the keyboard and Publishing to Twist!
---------------------------
[i] go straight
[j] turn left
[l] turn right
[k] go backwards

[q]/[z] : increase/decrease max speeds by 10%
[w]/[x] : increase/decrease only linear speed by 10%
[e]/[c] : increase/decrease only angular speed by 10%


I modified some lines of code in order to adapt the process on my needs. This algorithm is based on the modification of a Dictionary. Dictionary is adata type similar to a list and it contains a collection of objects. These objects are idexed with the key/value pair. In this case the contained value are the linear and angular velocity combinations that the robot should assume after that an input occurrs. Such collision avoidance algorithm just uses the.pop(key) method that removes and returns the element idexed by key of the dictionary. In this way, when the distance between the robot and the obstacle is less than a threshold value, a command is disabled just by popping such index from the dictionary. Moreover, by this way the robot cannot move in the direction in which the obstacle is detected. For implementing such algorithm, I needed the subscription to the /scan topic. This subscription allowed me to detect the obstacles in the simulation environment but also their position with respect to the robot by slicing the LaserScan array in three parts: front, left and right direction:

Here's a picture that represent how I sliced the laser scan span:



<img width="278" alt="Immagine-laser-scanner" src="https://user-images.githubusercontent.com/92534255/170269974-0002f9bb-fc4a-43bf-9298-6992bd1d8cfb.png">




def clbk_laser(msg):
		# Callback for detecting obstacles from laser scan
		global ok_left		#This variable is set to True when there are no obstacles on the left of the robot
		global ok_right		#This variable is set to True when there are no obstacles on the right of the robot
		global ok_straight	#This variable is set to True when there are no obstacles in front of the robot
		
		right = min(min(msg.ranges[0:143]), 1) # Detects obstacles on the right of the robot
		front = min(min(msg.ranges[288:431]), 1) # Detects obstacles in front of the robot
		left = min(min(msg.ranges[576:719]), 1) # Detects obstacles on the left of the robot
		
		if right != 1.0: # No obstacles detected on the right at a distance less than 1 meter
				ok_right =False
		else: # Obstacle detected on the right of the robot
				ok_right =True
		if front != 1.0: # No obstacles detected in the front direction at a distance less than 1 meter
				ok_straight =False
		else: # Obstacle detected in front of the robot
				ok_straight =True
		if left != 1.0: # No obstacles detected on the left at a distance less than 1 meter
				ok_left =False
		else: # Obstacle detected on the left of the robot
				ok_left =True



Then, I considered all the eight combination that the robot could face:


<img width="245" alt="Immagine-robot-possible-position" src="https://user-images.githubusercontent.com/92534255/170270004-a0759888-8edd-42cd-8b1b-75b0e55380d7.png">




and finally I popped the dictionary indexes that would have moved the robot towards the detected obstacle:
# Function that removes commands from the dictionary when an obstacle is detected. 
#In this way, when an obstacle is detected, the movement in that direction is disabled
def pop_dict(dictionary):


		global ok_left
		global ok_right
		global ok_straight
		# All the cases the robot could face
		if not ok_straight and not ok_right and not ok_left: # Obstacles in every direction
				# Disable all the three commands
				dictionary.pop('i') # Disable the front movement
				dictionary.pop('j') # Disable the left turn movement
				dictionary.pop('l') # Disable the right turn movement
		elif not ok_left and not ok_straight and ok_right: # Obstacles on the left and in the front direction, so right direction is free
				dictionary.pop('i') # Disable the front movement
				dictionary.pop('j') # Disable the left turn movement
		elif ok_left and not ok_straight and not ok_right: # Obstacles on the right and in the front direction, so left direction is free
				dictionary.pop('i') # Disable the front movement
				dictionary.pop('l') # Disable the right turn movement
		elif not ok_left and ok_straight and not ok_right: # Obstacles on the right and on the left, so the front direction is free
				dictionary.pop('l') # Disable the right turn movement
				dictionary.pop('j') # Disable the left turn movement
		elif ok_left and not ok_straight and ok_right: # Obstacles only in the front direction, so the left and right directions are free
				dictionary.pop('i') # Disable the front movement
		elif not ok_left and ok_straight and ok_right: # Obstacles only in the left direction, so the front and right directions are free
				dictionary.pop('j') # Disable the left turn movement
		elif ok_left and ok_straight and not ok_right: # Obstacles only in the right direction, so the front and left directions are free
				dictionary.pop('l') # Disable the right turn movement


