This file describes the work done and steps to perform test on the filters package.

"https://github.com/swatifulzele/filters/tree/ros2_devel"

Work Done by referring ROS1 lunar-devel branch filters package folder. 
1) Migrated all header and source files into ROS2 style 
2) Migrated yaml files into ROS2 style  
3) Migrated .launch files into launch.py in ROS2 style
4) Migrated CMakeLists.txt and package.xml in ROS2 style

TESTING:********
Here launch files are used independently.Following are the steps to run the test cases independently:
 
1. Set the path  
a) source /opt/ros/bouncy/setup.bash
b) source ./install/setup.bash 
c) source ~/ros2_launch_ws_master/install/setup.bash 

2. To run Test cases related to the mean filter use following command:
ros2 launch filters test_mean.launch.py

3. To run Test cases related to the median filter use following command:
ros2 launch filters test_median.launch.py

4. To run Test cases related to the test_params use following command:
ros2 launch filters test_params.launch.py

5. To run Test cases related to the transfer function filter use following command:
ros2 launch filters test_transfer_function.launch.py

6. To run Test cases related to the filter chain  use following command:
ros2 launch filters test_chain.launch.py

NOTE:- 1)Consider master branch of launch package to test .launch.py files. 
       2)colcon test does not work as launch.py files can not be executed/added with CMakeLists.txt as of now.
       3)Each test/launch.py files have been tested independently.
