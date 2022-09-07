# tiago_social_experiments
Package that runs TiAGO robot system to perform specific experiments.

Once workspace has been build, launch exemplary experiment (`012` scenario) with (Ubuntu 18 & ROS Melodic):

```bash
cd ~/ros_workspace/ws_social_navigation
source /usr/share/gazebo-9/setup.sh && source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch tiago_social_experiments 012.launch
```

One may want to launch `012` world without robot - it can be done with:

```bash
roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find tiago_sim_integration)/worlds/lab_012_v2_actor.world
```

## Evaluation

To evaluate the local planner, use metrics calculator [`move_base_benchmark`](https://github.com/rayvburn/move_base_benchmark). Once `csv` with experiment data is generated follow steps described in [`postprocessing`](https://github.com/rayvburn/move_base_benchmark/tree/melodic-devel-benchmark/postprocessing).
