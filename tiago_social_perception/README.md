# tiago_social_perception
Resources to provide necessary perception skills to social robot TIAGo.

This package highly relies on the SPENCER people tracking stack.

## Run

Run the `launch` file that provides TIAGo integration with the SPENCER tracking packages:
```bash
roslaunch tiago_social_perception tiago_spencer_tracking.launch
```

One may add `visualization:=false` if `rviz` visualization of robot perception is not required.

## Troubleshooting

### Lack of `upper body detector` subscriptions

Upper body detector does not subscribe RGBD topics. By default, sensor topics are:

```console
 * /spencer/sensors/rgbd_front_top/depth/camera_info [sensor_msgs/CameraInfo]
 * /spencer/sensors/rgbd_front_top/depth/image_rect [sensor_msgs/Image]
 * /spencer/sensors/rgbd_front_top/ground_plane [rwth_perception_people_msgs/GroundPlane]
```

Check the topics with:
```bash
rosnode info /spencer/perception_internal/people_detection/rgbd_front_top/upper_body_detector
```

Once these 3 topics above (or similarly named) are connected to the `upper body detector` node then most likely RGBD detections are not fused with laser detections. Set `rgbd` to `true` and safely `laser_low_confidence_detections` to `true`:

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
