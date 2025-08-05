# gentact_ros_tools

Install this ros2 package to visualize GenTact sensors as URDFs

# To visualize the sensor on its own:
```bash
ros2 launch gentact_ros_tools self-cap
```

# To calibrate the sensor
```bash
ros2 launch gentact_ros_tools self-cap_calibration.launch.py
```

```bash
noetic \
    foxy \
    ros2 run ros1_bridge dynamic_bridge
```

Then the controller
# 

# Example of compiling a urdf:
```bash
ros2 run xacro xacro urdf/robot/fr3_full_skin.xacro ee_xacro_file:=~/ros2_ws/src/gentact_ros_tools/urdf/end_effectors/sphere_ee.xacro > urdf/compiled/fr3_spheretip.urdf
```

# Running Avoidance Controller 
```bash
cd col_avoid_catkin_ws
source devel/setup.bash
roslaunch hiro_collision_avoidance robot.launch is_sim:=true
```

```bash
cd col_avoid_catkin_ws
source devel/setup.bash
roslaunch hiro_collision_avoidance self-cap-experiment.launch robot_movement:=circle robot_controller:=ding obstacle_type:=line_to_ee
```
can switch robot_movement: to static or circle
can also switch robot_controller:= to ding or flacco or hiro

```bash
cd ros2_ws
local_ws
ros2 launch gentact_ros_tools self-cap_avoidance.launch.py
```

```bash
cd ros2_ws
noetic
foxy
ros2 run ros1_bridge dynamic_bridge 
```

```bash
cd ros2_ws
local_ws
ros2 run gentact_ros_tools tuner 0.001
```

to rebuild ~/col_avoid_catkin_ws use ```catkin build```


# ML predictions:
```bash
ros2 launch gentact_ros_tools self-cap_predictions.launch.py
```