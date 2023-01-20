# ias_unitree_camera

This package publish Unitree Go1 cameras on a ROS network.

## Usage

To use this package put it inside a ```catkin_ws/src``` and build it (e.g. ```catkin build```). \
To start camera node use the ```unitree_camera.launch``` file in the following way:

```bash
  roslaunch ias_unitree_camera unitree_camera.launch camera_name:=CAMERA_NAME publish_rect_rgb:=true publish_raw_rgb:=false publish_depth:=true publish_pointcloud:=false publish_camera_info:=true
```

```CAMERA_NAME``` is needed to select the camera you want to start and it is **MANDATORY**. \
The available cameras for the Go1 are ```{face, chin, left, right, rearDown}```. \
Be aware of the robot configuration because the cameras are connected to specific PCs and they can be launched only from that computers; for example, in our Go1 configuration face/chin are connected to the first nano (192.168.123.13), left/right are connected to the second one (192.168.123.14) and rearDown are connected to the last nano (192.168.123.15).

Other useful launch params are:
- ```fps``` (int) camera fps
- ```image_width``` (int) camera width 
- ```image_height``` (int) camera heigth
- ```ros_rate``` (int) rate of message publishing 
