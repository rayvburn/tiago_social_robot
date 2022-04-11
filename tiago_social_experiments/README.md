# tiago_social_experiments
Package that runs TiAGO robot system to perform specific experiments.

Launch experiment (Ubuntu 18 & ROS Melodic) with:

```bash
cd ~/ros_workspace/ws_social_navigation
source /usr/share/gazebo-9/setup.sh && source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch tiago_social_experiments kkr.launch people_sim:=true perception:=true local_planner:=hubero publish_goal:=true
```

Launch `KKR` world with:

```bash
roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find tiago_sim_integration)/worlds/lab_012_v2_actor.world
```

## Evaluation

To evaluate local planner, [MRPB 1.0](https://github.com/NKU-MobFly-Robotics/local-planning-benchmark) by Wen et al. is used. Once `csv` with experiment data is generated follow these steps:

```bash
cd <ROS_WORKSPACE_DIRECTORY>
source devel/setup.bash
cd $(rospack find move_base_benchmark)/doc
g++ metrics.cpp -o metrics_evaluator
./metrics_evaluator <PATH_TO_LOG_FILE>
```

After that planner's score should be printed in the console.
