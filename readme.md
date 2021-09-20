NE Five
==

NE Five is a mobile robotics platform heavily inspired by [Johnny Five](https://en.wikipedia.org/wiki/Short_Circuit_(1986_film)) and is an evolution of the "MacFeegle Prime" robot originally developed for [PiWars](www.piwars.org) by [Keegan Neave](https://wwww.twitter.com/neaveeng). Work has been kindly supported by my [Patrons](https://www.patreon.com/neaveeng)!

A video of the current version of the robot can be found on my [YouTube channel](https://www.youtube.com/neaveeng), click the preview to play.  
[![What Went Wrong At PiWars 2021?](https://img.youtube.com/vi/SolxEP_HlIM/0.jpg)](https://www.youtube.com/watch?v=SolxEP_HlIM "What Went Wrong At PiWars 2021? - click to play video!")


NE Five was designed to be a more robust and feature complete robot building on the lessons learned from MacFeegle Prime. In this first release are all the files you need to run a basic simulation of the robot in [Gazebo](http://gazebosim.org/).

The simulation includes:
- Two 7dof arms
- Two simulated cameras for stereo vision
- Two IMU, one on the chassis and one on the base of the head

Instructions
--
These instructions are for the simulation only and will be updated once the other packages are brought online. 
1. Install [ROS Noetic](http://wiki.ros.org/noetic/Installation) Full Desktop
2. [Create a ROS catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Clone this repo in to the `src` folder
4. Run `catkin_make` in your [workspace root](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
5. Run `roslaunch nefive_description start_nefive.launch`

You should see a Gazebo window open with a simulated robot inside.  
<img src="https://neave.engineering/wp-content/uploads/2021/09/gazebo-running.png" width="300">

Keyboard Teleop
--
The simulation includes a differential drive controller, you can drive the robot around using keyboard teleop by running this in a new terminal:  
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=nefive/cmd_vel`

Head and Arm Control 
--
Control of the joints in the arms and head are available by publishing to the suitable topic:  
`rostopic pub /nefive/head_pan_joint_position_controller/command std_msgs/Float64 "data: -0.2"`

A list of all actuated joints can be found in `config/nefive_control.yaml`

Simulated Sensors
--
The robot includes two simulated cameras and two simulated IMUs, data for these can be found on the following topics:
- /nefive/leftcamera
- /nefive/rightcamera
- /base/imu/data
- /head/imu/data

Packages
--
- nefive (Metapackage)
- nefive_description 

Known Issues
--
This is the first URDF I've made from scratch and I'm still very much learning, there are a few things I've not dug in to yet and likely plenty more besides. PRs welcome!
- Over a certain velocity the robot rips itself apart...
- The arms haven't been fully tested nor their limits set
- Gazebo complains about the PIDs of the front wheels not being set but if I set the the robot self destructs...

Future Plans
--

The design of the physical robot is almost ready for release, there are a [few things](https://www.youtube.com/watch?v=SolxEP_HlIM) I want to fix beforehand however.

- Change the position of the Pi Zero enclosure
- Integrate the eyebrow servos
- MoveIt configuration
- LED eye ring control
- Add the CAD models to the simulation

Patron Support
--
A massive thank you to my [Patrons](https://www.patreon.com/neaveeng) who make this work possible, if you are able please consider supporting me on Patreon as it will help continue development of this robot.

Patrons
- Angie Neave
- John Neave 
- Bodger

Supporters
- Mark Long
- Adam Gilmore
- Bret Colloff
- David Shrive
- Sarah Cooper-Pinchbeck

Followers
- Mandy Berry
- Shelagh Lewins
- Alexander Gutenkunst
- Alister Perrott
- Andy Batey
- Eva Blake
- Cara S.
- Dave Booth 
- Pheonix Labs
