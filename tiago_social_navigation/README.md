# tiago_social_navigation
Resources to run social navigation on the TIAGo robot.

## Configuration parameters
The `config` directory contains configuration files based on `PAL`'s `pal_navigation_cfg_tiago` package.

### `costmap_converter` parameters
- `costmap_converter_plugin` (observations are related to default parameter configurations)
    - `CostmapToPolygonsDBSMCCH`: produces a lot of obstacles (separate object) which affects `HUMAP`'s traj. generation performance
    - `CostmapToPolygonsDBSConcaveHull`: produces moderate amount of obstacles but ignore a lot of single points that are obstacles; in the end makes traversing through corners harder
    - `CostmapToLinesDBSRANSAC`: produces massive amount of obstacles
    - `CostmapToLinesDBSMCCH`: produces approximately 3x more obstacles than `DBSRANSAC`

## Launching different local trajectory planners

NOTE: [`srl_eband`](https://github.com/rayvburn/srl_eband_local_planner) planner should be launched only with a specific configuration of costmap layers, i.e., [`socially_normative`](https://github.com/rayvburn/socially_normative_navigation), e.g.,:

```sh
roslaunch tiago_social_navigation navigation_base.launch local_planner:=srl_eband costmap_contexts:=socially_normative
```
