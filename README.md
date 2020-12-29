# glmapping is out of maintenance  <br /> please check our new mapping kit [MLMapping](https://github.com/HKPolyU-UAV/MLMapping)

### Introduction
This mapping kit is a 3D occupancy voxel map designed for the MAV/mobile robot navigation application. Currently, most of the navigation strategies are a combination of global planning and local planning algorithms. Global planning focuses on finding the least cost path from the current position to the destination. Furthermore, local planning is to replan the trajectory to avoid obstacles. This mapping kit processes the perception information separately. The globalmap, on the cartesian coordinate system, is a probability occupancy map. While the localmap, on cylindrical coordinates system, has an excellent dynamic performance. The mapping kit also supports the projected 2D occupancy grid map and ESDF(Euclidean Signed Distance Field) map output.

### Demo
<img src="others/mapping.gif" width="400">


### Usage
Clone this repository to catkin src folder say: ~/catkin_ws/src
````
cd ~/catkin_ws/src
git clone https://github.com/Ttoto/glmapping.git
````
Install 3rd Part library
````
cd catkin_ws/src/glmapping/3rdPartLib/
./install3rdPartLib.sh
````
Compile
````
cd ~/catkin_ws/
catkin_make
````
Download the [Dataset](https://drive.google.com/file/d/1AF0zBQUizYWccYE9hravpHns8beP1a0Z/view?usp=sharing) into the bag folder <br />
decompress the rosbag
````
rosbag decompress glmapping_test.bag
````
run 
````
roslaunch glmapping bag.launch
````
### Maintainer
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />

