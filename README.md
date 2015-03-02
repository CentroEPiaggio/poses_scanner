# Poses Scanner Nodes
A series of nodes to acquire a database of poses for pose estimation procedure
## Setup
### Software Setup
You require a working Ros(indigo) repository with the following packages:
 - Poses scanner node (included)
 - Turntable interface node (included)
 - rgbd_lwr (included)
 - openni2 (install with apt or download from [Ros Drivers](https://github.com/ros-drivers))
    * openni2_launch
    * openni2_camera
    * rgbd_launch
 - kuka_lwr (from [CentroPiaggio repositories](https://github.com/CentroEPiaggio/kuka-lwr))
 - [scene_filter](https://bitbucket.org/Tabjones/scene_filter)
 
Execute `catkin_make` inside the repository root to update any changes.
### Hardware Setup
- Turntable
- Power supply of 7.5V to turn on turntable's servo.
- Asus Xtion Pro Live.
- Kuka LWR right arm.
- Poses scanner tool loaded and mounted on right arm.
`TODO` more info in the near future

## Usage (WORK IN PROGRESS)
To launch the poses scanner execute:
`roslaunch poses_scanner_node poses_scanner_node.launch`

At the start of each session make sure you acquired the various table transformations with:
`rosservice call /poses_scanner_node table`
You don't need to launch this service again, unless you move the turntable from its position or you move the Asus Xtion from the Kuka arm.

To actually acquire poses of an object use the command:
`rosservice call /poses_scanner_node acquire 'name: OBJNAME lonpass:LONPASS'`


