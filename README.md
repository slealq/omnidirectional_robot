# omnidirectional_robot
Ros module for Arcos Lab omnidirectional robot.

# Dependencies 
You'll need to install rospkg (In case you don't already have it) and pyserial in order to run this code.

    pew new ros
    pip install pyserial rospkg catkin_pkg numpy
    
**NOTE:** You'll need to use pew workon ros each time you want to use this package. Read more about pew in case you don't know what it is: https://pypi.org/project/pew/
  
# Installation
In order to use this ros driver, you need to clone this code in your catkin workspace:
  
    cd ~/catkin_ws/src
    git clone https://github.com/slealq/omnidirectional_robot/ 
    cd ../
    catkin_make
  
# Usage
You can bring up the ros node using this: 
  
    roslaunch omnidirectional_node bringup.launch
    
**NOTE:** In order to move the omnidirectional robot with a ps4 controller, we advice to use this git: https://github.com/solbach/ps4-ros

# Using PS4 controller
Follow the instructions on ps4-ros repo. Note that ps4-ros is only a repo for publishing joy messages to ros. In order to move the base, you'll need this code: https://github.com/ros-teleop/teleop_twist_joy, which will convert joy messages to twist, and publish them in cmd_vel, which is the topic omnidirectional driver listens to.

    sudo apt-get install ros-indigo-teleop-twist-joy
    
# Example usage with PS4 controller
Assuming you already have ps4-ros and teleop-twist-joy running: 

1. Enable roscore
    
        roscore
    
2. First, enable python workspace with pew. Then pair your ps4 controller: 

        pew workon ros
        
        ds4drv
        
3. Pay attention to what device is it. Ex: /dev/input/jsX

4. On another terminal, edit the launch file of teleop_twist_joy (located in teleop_twist_joy/launch/teleop.launch). Change the 

        roscd teleop_twist_joy/launch/
        
        emacs -nw teleop.launch
        
5. Edit the third line. Change default="/dev/input/js0" with your ps4 controller: 

        ~~ default="/dev/input/js1"
        
        defaultl="/dev/input/jsX" 

6. On a third terminal, bring un teleop twist joy: 

        pew workon ros 

        roslaunch teleop_twist_joy teleop.launch
        
7. On a fourth terminal, bring up ps4 ros node

        pew workon ros
        
        rosrun ps4_ros ps4_ros
        
8. You should now have a basic controller, for your omnidirectional base. The bindings are located in teleop_twist_joy/config
      
        
