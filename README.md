# tiago_social_robot

Packages for social navigation based on TIAGo robot.

## Run

Run HuBeRo example with sourced TIAGo packages (to spawn the robot in simulation):

Start Gazebo simulation:
```bash
cd ~/ros_workspace/ws_social_navigation
source /usr/share/gazebo-9/setup.sh && source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=living_room rviz:=false
```

Spawn TIAGo robot:
```bash
cd ~/ros_workspace/ws_social_navigation
source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch tiago_social_bringup gazebo_spawn.launch
```

Run navigation modules for TIAGo:
```bash
cd ~/ros_workspace/ws_social_navigation
source ../ws_tiago/devel/setup.bash && source devel/setup.bash
```
- localization
  - if `map_server` is already running
    ```bash
    roslaunch tiago_social_navigation gazebo_nav_base.launch state:=localization map:=none
    ```
  - if new map should be loaded (map path is just an example)
    ```bash
    roslaunch tiago_social_navigation gazebo_nav_base.launch state:=localization map:=$(rospack find hubero_bringup_gazebo_ros)/maps/living_room.yaml
    ```

- mapping (SLAM)
  ```bash
  roslaunch tiago_social_navigation gazebo_nav_base.launch state:=mapping map:=none
  ```

Alternatively, navigation modules can be ran using this command (will also open special `rviz` config provided by `PAL`):
```bash
roslaunch tiago_social_navigation gazebo_nav_tiago.launch map:=none
```

Custom rviz config:
```bash
rviz -d $(rospack find tiago_social_navigation)/rviz/tiago_navigation.rviz
```

NOTE: `tiago_social_navigation/gazebo_nav_tiago.launch` can only be used for localization.

## Benchmarking
First install dependencies of `NKU-MobFly-Robotics/local-planning-benchmark`:

```bash
sudo apt install libceres-dev
```

## Troubleshooting

### Frame names
This package does not fully overhaul PAL's launch ecosystem but only modifies some parts to make it usable with external maps, to allow easier override of parameters.
User must beware of frame names as these are hard-coded in multiple places.
Generally, frames should not be touched and valid setup consists of such frame names:
- robot base frame -> `base_footprint`
- odometry frame -> `odom`
- map frame -> `map`

### Stuck at AMCL launch
If you're stuck on AMCL launched to this stage:

```console
[ INFO] [1646218893.974936024]: Done initializing likelihood field model.
```

then start with a system reset. At the next startup you'll probably see this kind of output:

```console
[ INFO] [1646219077.821685430, 29.811000000]: Initializing likelihood field model; this can take some time on large maps...
[ INFO] [1646219078.428821750, 29.837000000]: Done initializing likelihood field model.
[ERROR] [1646219079.735113000, 30.112000000]: Lookup would require extrapolation into the past.  Requested time 30.095000000 but the earliest data is at time 30.107000000, when looking up transform from frame [base_footprint] to frame [map]
[ WARN] [1646219079.756207959, 30.117000000]: global_costmap: Pre-Hydro parameter "map_type" unused since "plugins" is provided
[ INFO] [1646219079.758755195, 30.117000000]: global_costmap: Using plugin "static_layer"
[ INFO] [1646219079.776424438, 30.121000000]: Requesting the map...
[ INFO] [1646219080.307251280, 30.221000000]: Resizing costmap to 712 X 650 at 0.020000 m/pix
[ INFO] [1646219081.883263635, 30.321000000]: Received a 712 X 650 map at 0.020000 m/pix
[ INFO] [1646219081.892391411, 30.323000000]: global_costmap: Using plugin "obstacle_laser_layer"
[ INFO] [1646219081.903287792, 30.324000000]:     Subscribed to Topics: base_scan
```

This mostly happens once robot is spawned along with navigation modules and then navigation modules are killed and then started again.

### Chaining multiple ROS workspaces

Assuming that you have workspaces designated for:

- robot software and controllers
- perception stuff
- social navigation stuff that is under heavy development

you have to compile workspaces in that order:

- robot software

  ```bash
  cd ~/ros_workspace/ws_tiago
  source /opt/ros/melodic/setup.bash
  catkin build
  source devel/setup.bash
  ```

- perception

  ```bash
  cd ~/ros_workspace/ws_perception
  source ../ws_tiago/devel/setup.bash
  catkin build
  source devel/setup.bash
  ```

- development stuff

  ```bash
  cd ~/ros_workspace/ws_social_navigation/
  source ../ws_perception/devel/setup.bash
  catkin build
  source devel/setup.bash
  ```

That's it. [Reference](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying).
