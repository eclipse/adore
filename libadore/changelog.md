# Change Log

## 0.1.0 - 2020-04-15
* Initial Eclipse release
* adore/sim: Simulation tools
* adore/fun: Automated driving control functions, qpOASES based maneuver planner, feed-back control
* adore/apps, new applications:
    * borderbird.h - simple birds-eye-view plotting application connecting to plotlabserver
    * feedbackcontroller.h - trajectory tracking
    * lane_following_behavior.h - application of qpOASES based maneuver planner to lane following
    * monitor0.h - prototype application for evaluation of important predicates for automatic simulation evaluation
    * objectdetectionmodel.h - application with simple object detection model
    * plainxmlexporter.h - prototype application for exporting of maps to sumo plain xml format
    * test_lc_trajectory_planner.h - test application for lane change maneuver planning with lots of visualizations
    * test_trajectory_planner.h - test application for lane following maneuver planning with lots of visualizations
    * trafficlighsim.h - prototype application for stand-alone traffic light phase simulation
    * vehiclemodel.h - application with vehicle model for use in simulation experiments

## 0.0.1 - 2019-11-27
* Initial release
* adore/views: Data abstraction for nominal automated driving maneuver planning
* adore/env: BorderBased realization of LaneFollowingView and LaneChangeView
* adore/mad: Function library, nonlinear regression for fitting of road coordinate system, interface to qpOASES for dynamic optimization problems
* adore/if_xodr: OpenDrive file loader
* adore/apps, new applications:
    * map_provider.h - streaming of static environment information in vehicle vicinity
    * navigation.h - global navigation module / graph search

