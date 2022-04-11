# tiago_social_experiments
Package that runs TiAGO robot system to perform specific experiments.

Launch experiment (Ubuntu 18 & ROS Melodic) with:

```bash
cd ~/ros_workspace/ws_social_navigation
source /usr/share/gazebo-9/setup.sh && source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch tiago_social_experiments kkr.launch people_sim:=true perception:=true local_planner:=hubero
```

Launch `KKR` world with:

```bash
roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find tiago_sim_integration)/worlds/lab_012_v2_actor.world
```