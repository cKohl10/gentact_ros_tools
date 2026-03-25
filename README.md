# gentact_ros_tools

Install this ros2 package to visualize GenTact sensors as URDFs

NEW: 

# Run hybrid skin (ToF only so far) -- launches foxglove as well 

```bash 
ros2 launch gentact_ros_tools_hybrid sensors.launch.py config:=hybrid.yaml
```

### TODO:
- Add functionality to tof_pub to accept multiple ports for one skin
- Add pointcloud filtering



# To run the spad skin + sensors:
```bash
ros2 launch gentact_ros_tools_hybrid spad2.launch.py config:=spad.yaml
```
# To run controller (to get the robot model working):

Make sure your controller is on before running this

```bash
ros2 run gentact_ros_tools_hybrid franky_xbox
```
# To launch sensors (outside of this package):

Make sure the sensors are powered on before running this. If you don't get data outputting, try resetting your microcontroller or using a different battery.  

```bash 
ros2 run udp_tof_listener udp_grid_listener_array 
```

```bash
ros2 run pointcloud talker
```
