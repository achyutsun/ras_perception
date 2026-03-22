# Perception
Perception, Semantic Landmark Extraction and Classification using 3D point cloud preprocessing, RANSAC, Normal Estimation, HSV Color Space implementation.

## Steps to Run
```
$ cd ~/ros_ws
$ colcon build --packages-select cylinder_detection
$ source install/setup.bash
$ ros2 launch cylinder_detection cylinder_launch.py
```
In another Terminal
```
$ cd to_folder_with_rosbag
$ ros2 bag play rgbd_bag_2_0.mcap --loop --rate 0.5
```
In third Terminal
```
$ ros2 topic echo /oakd/points
or
$ tmux
$ fte
select /oakd/points
```

## Demo Video
https://github.com/user-attachments/assets/62a8cba3-7273-4028-8b32-8d86664056f9

<img width="1208" height="743" alt="Screenshot" src="https://github.com/user-attachments/assets/8a567426-399e-4419-8b4b-307ea67e9d4f" />
Figure: Screenshot of Rviz
