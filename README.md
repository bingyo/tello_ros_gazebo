# tello_ros
ROS Tello driver and gazebo

### Run
#### Gazebo
```
roslaunch tello_driver indoor_slam_gazebo.launch
rosservice call /enable_motors "enable: true"
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
rosrun tello_autonomous circle.py
```

#### Tello
```
roslaunch tello_driver orb.launch
rosrun tello_autonomous circle.py
```

### todo

- Fix intrinsic and extrinsic parameters of camera
- Fix scale
- Show path plan 
- Add calibration pose

### Reference

[yuokamoto/ros_tello](https://github.com/yuokamoto/ros_tello)

[appliedAI-Initiative/orb_slam_2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros)

[ dji-sdk/Tello-Python](https://github.com/dji-sdk/Tello-Python)
