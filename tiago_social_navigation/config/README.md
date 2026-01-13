# Navigation parameters

See [`pal_navigation_cfg_public/pal_navigation_cfg_tiago/config`](https://github.com/pal-robotics/pal_navigation_cfg_public/tree/master/pal_navigation_cfg_tiago/config) for reference.

## Tuning hints

- planners using `SimpleTrajectoryGenerator` must have acceleration limits set according to the minimum velocity and control frequency. For example, if the DWA planner has `min_vel_trans` of 0.1 m/s and operates with a 10 Hz frequency, the smallest `acc_lim_x` should be set to 1.0 m/s^2. Otherwise, any valid velocity cannot be generated for a standing mobile base.

## Notes

Note that TIAGo's velocity multiplexer has a timeout of 500 ms. Choose controller frequency according to that.
