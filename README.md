# simple_car
## Enviroment 
* Ubuntu16.04 + Ros kinetic
## Dependence
1) **ros seiral driver**  
``sudo apt-get install ros-kinetic-serial-*``
2) **ros message package**  
`` sensro_msgs、 geometry_msgs、 grid_map_msgs、 nav_msgs
``  
Usually, these message packages will be installed automaticly when you install ROS. 
Otherwise, you can use command `sudo apt-get install ros-kinetic-[package name]` to install them

## How to use
1) create a ros workspace folder  
`mkdir ~/catkin_ws/src`
2) download simple_car code from github: `https://github.com/ytiang/simple_car` , 
then put the code into `catkin_ws/src/` folder
3) download rplidar driver from github: `https://github.com/robopeak/rplidar_ros`,
 then put the code into `catkin_ws/src/` folder
4) `cd ~/catkin_ws`  
   `catkin_make`  
   `source devel/setup.bash`
5) plug usb device in, then use command  
`bash ~/catkin_ws/src/simple_car/script/setup_device.bash`
6) `roslaunch simple_car simple_car_node.launch`