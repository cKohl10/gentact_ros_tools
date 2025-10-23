## Purpose of Packages

- franky sends joint states to the FR3
- col_avoid_catkin_ws houses the avoidance controllers
- gen_tact_ros_tools creates an obstacle using the sensor outputs 

## To Run
Make sure robot joint's unlocked and FCI activated
Make sure sensor is on robot and connected 

bash
cd col_avoid_catkin_ws 
source devel/setup.bash
roslaunch hiro_collision_avoidance robot.launch is_sim:=true

bash 
cd col_avoid_catkin_ws
source devel/setup.bash
roslaunch hiro_collision_avoidance self-cap-experiment.launch robot_movement:=circle robot_controller:=hiro obstacle_type:=line_to_ee

flacco and xbox do not work right now 

--to build: catkin_make

bash
cd ros2_ws
foxy
local_ws
ros2 run gentact_ros_tools tuner 0.003

bash
noetic
foxy
ros2 run ros1_bridge dynamic_bridge 

bash
cd ros2_ws
foxy
local_ws
ros2 run gentact_ros_tools franky_relay 


bash 
cd ros2_ws
foxy
local_ws
ros2 launch gentact_ros_tools self-cap_avoidance.launch.py 

## record
ros2 bag record -s mcap --all