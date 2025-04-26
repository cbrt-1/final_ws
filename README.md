# fri_final
```
cd ~/final_ws
git clone https://github.com/utexas-bwi/serial_for_ros2.git
cd ~/final_ws/serial_for_ros2/
mv serial ..
cd ..
cd serial
rm -rf build
mkdir build
cd build
cmake ..
make
```

After cloning, run the command below in order to clone the other dependencies.
```
vcs import src < ros2.repos
```
```
ros2 launch azure_kinect_ros_driver driver.launch.py
```
```
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/rgb/image_raw depth_topic:=/rgb_to_depth/image_raw camera_info_topic:=/rgb_to_depth/camera_info
```
```
ros2 run rviz2 rviz2
```