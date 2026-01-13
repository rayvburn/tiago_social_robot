# Algorithm-specific launch files

## `local_planner_<planner_name>`

Parameters passed to the main launch file for the navigation, i.e., the launch starting the `move_base` node, are not passed to the launch files for specific algorithms. Therefore, in some cases some hand-tuning will be necessary when, e.g., changing the default global planner etc.
