# Two NAO Robots Synchronised in ROS

## Info
I've been asked by the [University](https://commons.swinburne.edu.au/items/5195c967-61b3-4b1d-a8b6-7aaf560bc5f9/1/) to have two NAO robots talking about plagiarism. This was not very exciting, however, the idea of having 2 NAO robots connected was interesting.

This repository has two ROS nodes coded in C++ that coordinate 2 NAO robots to do a simple task such as counting together (alternating) or a speech about plagiarism. Artificial Intelligence on Academic Integrity (AI on AI)

### Two NAOs counting together (alternating)
[![Two NAO Robots Synchronised](https://i.imgur.com/eoRuakI.png)](https://vimeo.com/278641551 "Two NAO Robots Synchronised - Click to Watch!")

## Nodes
* two_nao_count_together: Node that coordinates the 2 robots when counting together (alternating) 
* two_nao_plagiarism_speech:  Node with the conversation of 2 robots about plagiarism


## Dependencies
* ROS (tested with ROS Kinetic Kame) 
* NAO ROS packages: 
  * http://wiki.ros.org/nao
  * http://wiki.ros.org/nao_bringup
  * http://wiki.ros.org/nao_apps


## How to Install
* `roscd`
* `cd ../src`
* `git clone  https://github.com/FelipMarti/two_nao_sync.git`
* `cd ..`
* `catkin_make`


## How to Run

### Two NAOs counting together (alternating)
Open 1 terminal
* `roscore`

Open another terminal and execute the roslaunch with the 2 different IP addresses (or hostnames) of the 2 robots. In my case nao.local is robot 1 and rosie.local is robot 2.
* `roslaunch two_nao_sync two_nao_count_together.launch nao_ip_1:=nao.local nao_ip_2:=rosie.local`


### Two NAOs talking about plagiarism (AI on AI)
#### Option 1
Open 1 terminal
* `roscore`

Open another terminal and execute the roslaunch with the 2 different IP addresses (or hostnames) of the 2 robots: `nao_ip_1` and `nao_ip_2`. In my case `nao.local` is robot 1 and `rosie.local` is robot 2.
* `roslaunch two_nao_sync two_nao_plagiarism_speech.launch nao_ip_1:=nao.local nao_ip_2:=rosie.local`

#### Option 2
Open 1 terminal
* `roscore`

Open another terminal and execute the roslaunch for the first robot. The two variables should be filled in: the ip address `nao_ip`; and the number of the robot `ROBOT`. In my case `nao.local` and `r1`
* `roslaunch two_nao_sync single_nao_with_req.launch nao_ip_1:=nao.local ROBOT:=r1`

Open another terminal and execute the roslaunch for the first robot. The two variables should be filled in: the ip address `nao_ip`; and the number of the robot `ROBOT`. In my case `rosie.local` and `r2`
* `roslaunch two_nao_sync single_nao_with_req.launch nao_ip_1:=rosie.local ROBOT:=r2`

Open another terminal and execute the roslaunch of the AI on AI code. The two IP addresses (or hostnames) of the 2 robots should be filled in:  `nao_ip_1` and `nao_ip_2`. In my case `nao.local` is robot 1 and `rosie.local` is robot 2.
* `roslaunch two_nao_sync two_nao_only_plagiarism_speech_node.launch nao_ip_1:=nao.local nao_ip_2:=rosie.local`

## Future Improvements
* There should be 1 node for each robot, and then connect them via topics (Publisher-Subscriber)
* Change the voice pitch of the robots, to make them sound different
* ...

