# multirobot
Forked from https://github.com/emrecanbulut/multirobot
Copy the multirobot folder under your catkin workspace source directory (e.g ```~/catkin_ws/src/multirobot```) 
To run, open up a new terminal and type ```roslaunch multirobot gazebo_multirobot.launch``` , this should run the gazebo showing 2 robots alongside.

```robot1.py``` is the sample controller code for thief and ```robot2.py``` is the same for the police. For now, there is no difference in between them (except the node names). To make these codes work, first ```cd``` into the multirobot directory, make them executable (```chmod +x robot1.py```) and run by ```./robot1.py``` .
This would make the codes work.

There are several issues coming with multiple robots; for example they should not move fast. I've tried my code from assignment-1 (which was working well with a single robot) and it moved really fast when it ran on multiple robot environment. So I've divided all of the velocity values by about 9. Second issue is, the robots are drifting. I hope I can solve these issues in time, but for now, try to move robots really slow.

If you are going to copy-paste your own code to robot1.py or robot2.py, make sure your node is publishing the same nodes as I did in robot1.py and robot2.py. Long in short, we were publishing navigation commands to ```/cmd_vel_mux/input/navi``` but for now, it must be ```/robot1/cmd_vel```. Review the code for ```scan``` and ```map``` publishment and subscribment paths, as well.

 
