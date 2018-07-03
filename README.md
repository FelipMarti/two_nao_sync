# Two NAO Robots Synchronised in ROS

## Info
I've been asked from the University to have two NAO robots talking about plagiarism. This was not very exciting, however, the idea of having 2 NAO robots connected was interesting.

This repository has two ROS nodes coded in C++ that coordinate 2 NAO robots to do a simple task such as counting together or a speech about plagiarism.


## Nodes
* two_nao_count_together: Node that coordinates the 2 robots when counting together. 
* two_nao_plagiarism_speech:  Node with the conversation of 2 robots about plagiarism.


## Dependencies
* ROS (tested with ROS Kinetic Kame) 
* NAO ROS packages: http://wiki.ros.org/nao
⋅⋅* http://wiki.ros.org/nao_bringup
⋅⋅* http://wiki.ros.org/nao_apps


## How to Install
* `roscd`
* `cd ../src`
* `git clone  https://github.com/FelipMarti/two_nao_sync.git`
* `cd ..`
* `catkin_make`


## How to Run it

### Two NAOs counting together
Open 1 terminal
* `roscore`

Open another terminal and execute the roslaunch with the 2 different IP addresses (or hostmane) of the 2 robots. In my case nao.local is robot 1 and rosie.local is robot 2.
* `roslaunch two_nao_sync two_nao_count_together.launch nao_ip_1:=nao.local nao_ip_2:=rosie.local`


### Two NAOs talking about plagiarism
Open 1 terminal
* `roscore`

Open another terminal and execute the roslaunch with the 2 different IP addresses (or hostmane) of the 2 robots. In my case nao.local is robot 1 and rosie.local is robot 2.
* `roslaunch two_nao_plagiarism_speech.launch nao_ip_1:=nao.local nao_ip_2:=rosie.local`


## Future Improvements
* Having 1 node for each robot, and connect the 2 robots with topics (Publisher-Subscriber) 
* Change the voice pitch of the robots, to make them sound different.
* ...

