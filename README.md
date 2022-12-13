# ROS driver for Continental ARS 408-21

This is a C++ ROS driver for Continental ARS 408-21 (or 404-21) automotive radar. The code is heavily based on the [ApexAI ROS2 Radar driver](https://gitlab.com/ApexAI/autowareclass2020/-/tree/master/code/src/09_Perception_Radar/Radar-Hands-On-Solution-WS) .

## Install and Build
make sure to clone in to your catkin workspace src folder

Execute from the root of the workspace:
`git clone https://github.com/mesmatyi/conti_radar_driver.git`\
`rosdep install --from-paths src --ignore-src -r -y`\
`catkin build` \
or if you want to build the package separately: `catkin build radar_conti`

## Run
source in: `source ~/your_catkin_workspace/devel.setup.bash` \
then run the node: `rosrun radar_conti radar_conti`

## Radar configuration CAN frames

More [CAN configuration frames](https://github.com/lf2653/myrepository)

Change between Object detection and Cluster detection:\
`
cansend can1 200#F8000000089C0000 // Objects detection with all extended properties`\
`cansend can1 200#F8000000109C0000 // Clusters detection with all extended properties
`

Alternatively configure with https://github.com/szenergy/conti_radar_configurator forked from https://github.com/lf2653/myrepository.
