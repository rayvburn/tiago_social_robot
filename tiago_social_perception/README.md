# tiago_social_perception
Resources to provide necessary perception skills to social robot TIAGo.

This package highly relies on the SPENCER people tracking stack.

## Run

Laser detectors:

**OUTDATED** already:
```bash
roslaunch tiago_social_perception people_detection_spencer.launch laser_topic:="/scan" laser_learned_detector_people_topic:="/detector/learned/people" laser_low_confidence_people_detections_topic:="/detector/blob/people"
```

Detectors:
```bash
roslaunch tiago_social_perception perception_tiago_lite.launch
```

Tracking:
```bash
roslaunch tiago_social_perception spencer_freiburg_people_tracking.launch rgbd:=true laser_low_confidence_detections:=true
```

Visualization:
```bash
rviz -d $(rospack find spencer_people_tracking_launch)/rviz/tracking-rgbd-laser.rviz
```

## Troubleshooting

### Lack of `upper body detector` subscriptions

Upper body detector does not subscribe sensors topics if tracking module is not operational. By default, sensor topics are:

```console
 * /spencer/sensors/rgbd_front_top/depth/camera_info [sensor_msgs/CameraInfo]
 * /spencer/sensors/rgbd_front_top/depth/image_rect [sensor_msgs/Image]
 * /spencer/sensors/rgbd_front_top/ground_plane [rwth_perception_people_msgs/GroundPlane]
```

~~What's more, it's crucial to start detection and tracking in the same launch file. Otherwise, upper body detector will end up not subscribing any sensor topic.~~ It seems to be an issue with args passed to launch (they were not defined), should be:

```bash
roslaunch tiago_social_perception spencer_freiburg_people_tracking.launch rgbd:=true laser_low_confidence_detections:=true
```

### Detections are not visually confirmed

Change `use_initiation_logic` in the `spencer_people_tracking/tracking/people/srl_nearest_neighbor_tracker/launch/nnt.launch` to `false`. Reference: [spencer_people_tracking#98](https://github.com/spencer-project/spencer_people_tracking/issues/98).

### Matching stopped working (detections operational)

Make sure that `filter_tracks_confirmed_by_HOG-31` has not died. It happens that this node crashes with log:

```console
[spencer/perception_internal/people_tracking/post_processing/filter_tracks_confirmed_by_HOG-31] process has died [pid 5788, exit code -11, cmd $HOME/ws_perception/devel/lib/spencer_tracking_utils/filter_visually_confirmed_tracks input_tracks:=/spencer/perception/tracked_persons_orientation_fixed output_tracks:=/spencer/perception/tracked_persons_confirmed_by_HOG composite_detected_persons:=/spencer/perception/detected_persons_composite __name:=filter_tracks_confirmed_by_HOG __log:=$HOME/.ros/log/5bf17c0a-9c1a-11ec-895d-a4c4949b3b6a/spencer-perception_internal-people_tracking-post_processing-filter_tracks_confirmed_by_HOG-31.log].
log file: $HOME/.ros/log/5bf17c0a-9c1a-11ec-895d-a4c4949b3b6a/spencer-perception_internal-people_tracking-post_processing-filter_tracks_confirmed_by_HOG-31*.log
[spencer/perception_internal/people_tracking/post_processing/filter_tracks_confirmed_by_upper_body-32] process has died [pid 5824, exit code -11, cmd $HOME/ros_workspace/ws_perception/devel/lib/spencer_tracking_utils/filter_visually_confirmed_tracks input_tracks:=/spencer/perception/tracked_persons_orientation_fixed output_tracks:=/spencer/perception/tracked_persons_confirmed_by_upper_body composite_detected_persons:=/spencer/perception/detected_persons_composite __name:=filter_tracks_confirmed_by_upper_body __log:=$HOME/.ros/log/5bf17c0a-9c1a-11ec-895d-a4c4949b3b6a/spencer-perception_internal-people_tracking-post_processing-filter_tracks_confirmed_by_upper_body-32.log].
log file: $HOME/.ros/log/5bf17c0a-9c1a-11ec-895d-a4c4949b3b6a/spencer-perception_internal-people_tracking-post_processing-filter_tracks_confirmed_by_upper_body-32*.log
```
