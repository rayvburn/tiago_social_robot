# tiago_social_navigation
Resources to run social navigation on the TIAGo robot.

## Configuration parameters
The `config` directory contains configuration files based on `PAL`'s `pal_navigation_cfg_tiago` package.

### `costmap_converter` parameters
- `costmap_converter_plugin` (observations are related to default parameter configurations)
    - `CostmapToPolygonsDBSMCCH`: produces a lot of obstacles (separate object) which affects `HuBeRo` traj. generation performance
    - `CostmapToPolygonsDBSConcaveHull`: produces moderate amount of obstacles but ignore a lot of single points that are obstacles; in the end makes traversing through corners harder
    - `CostmapToLinesDBSRANSAC`: produces massive amount of obstacles
    - `CostmapToLinesDBSMCCH`: produces approximately 3x more obstacles than `DBSRANSAC`
