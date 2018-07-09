# omnidirectional_robot
Ros module for Arcos Lab omnidirectional robot.

# Dependencies 
You'll need to install rospkg (In case you don't already have it) and pyserial in order to run this code.

    pew new ros
    pip install pyserial 
    pip install rospkg
  
# Installation
In order to use this ros driver, you need to copy this code in your catkin workspace:
  
    cd ~/catkin_ws/src
    git clone https://github.com/slealq/omnidirectional_robot/ 
    cd ../
    catkin_make
  
# Usage
You can bring up the ros node using this: 
  
    roslaunch omnidirectional_node bringup.launch
