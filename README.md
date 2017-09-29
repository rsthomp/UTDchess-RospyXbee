# UTDchess-RospyXbee
Python-xbee node for controlling UTD Chess Robot

This project allows for multiple robots (up to 32) to simultaneously use the Vicon Motion Capture system to track their position and inform their own respective control nodes. A video of the software in action can be found here: https://www.youtube.com/watch?v=6G1eWFsEJ-c

Instructions for Use: 

1. Turn on the Vicon, and open the Vicon Nexus program. Under “File > Data Management”, select the green “Chessbot” node. The subject pane should now have all the potential robots prepared.

2. Place all your robots on the field. Make sure that each one has the correct Xbee attatched, with an ID matching that of the robot's subject on the Vicon, and that the coordinator is plugged into the USB slot. Each robot should show up as a seperate subject on the Vicon Nexus program. Turn on all robots.

3. Open a terminal and enter “$ roslaunch vicon_bridge vicon.launch.” This node publishes to the “vicon/markers” topic with the names and positions of every marker on every subject seen by the Vicon.

4. Next, open the “~/lars-ros/src/FullChess/pointfile.yaml” file. Each robot should have an assigned target point and controller in this file. Select the points you would like to assign to each robot as a set of coordinates in meters, and select a controller (current options are: “PID_Template.py,” “PID_Repulsion_Template.py”). Then open a terminal and enter “$ rosparam load ~/lars-ros/src/FullChess/pointfile.yaml.” You can edit the points listed in this file and update them using this command at any point during the running of the program.

5. Open a terminal and enter “$ roscore.”

6. Open another terminal and enter “$ rosrun chessbot chess_manager.py.” The program should immediately begin searching for robots. It will list all addresses that it has found, and then 3 new windows will open per robot discovered. The program will immediately begin sending commands to the robots.

7. Cancel the program from the original terminal using “ctrl-C.” Robots will immediately stop moving, but will need to be turned off by hand. 

If:
Each robot's windows are opening, but are blank:
	Check your Vicon node to be certain it is still publishing robot locations.

The program ends without opening any windows:
	Check your Xbee communication. 

The main terminal window is throwing a 'KeyError':
	Check your Xbee communication.

A robot is twisting back and forth without centering on a direction:
	Try reversing the motor connections on the robot's chassis and restarting the program. 
