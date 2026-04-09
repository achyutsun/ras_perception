# Perception
Perception, Semantic Landmark Extraction and Classification using 3D point cloud preprocessing, RANSAC, Normal Estimation, HSV Color Space implementation.

## Steps to Run
In new Terminal   . Run set-ros-env sim first in all new terminal, then source
```
$ set-ros-env sim
$ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map oakd_rgb_camera_optical_frame
```
In second Terminal
```
ros2 run image_transport republish compressed raw \
--ros-args -r in:=/oakd/rgb/image_raw/compressed \
-r out:=/oakd/rgb/image_raw
```
In third Terminal. /oakd/points topic only gets published ater running Fourth Terminal run of ros bag play
```
$ ros2 topic echo /oakd/points
or
$ tmux
$ fte
select /oakd/points
```
In another Terminal or fourth Terminal
```
$ cd to_folder_with_rosbag
$ ros2 bag play rgbd_bag_2_0.mcap --loop --rate 0.5
```

In Fifth Terminal
```
$ cd ~/ros_ws
$ colcon build --packages-select cylinder_detection
$ source install/setup.bash
$ ros2 launch cylinder_detection cylinder_launch.py
```

## Demo Video
https://github.com/user-attachments/assets/62a8cba3-7273-4028-8b32-8d86664056f9

<img width="1208" height="743" alt="Screenshot" src="https://github.com/user-attachments/assets/8a567426-399e-4419-8b4b-307ea67e9d4f" />
Figure: Screenshot of Rviz
