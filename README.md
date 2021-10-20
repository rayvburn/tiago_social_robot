# tiago_scenarios
Package that runs TiAGO robot (simulation) system to execute certain scenarios

## Run exemplary world
Launch whole system with:

```bash
roslaunch tiago_scenarios navigation.launch
```

Or run robot simulation and navigation in separate tabs:

```bash
roslaunch tiago_scenarios robot_simulation.launch
roslaunch tiago_scenarios navigation.launch
```

## Configuration parameters
The `config` directory contains configuration files based on `PAL`'s `pal_navigation_cfg_tiago` package