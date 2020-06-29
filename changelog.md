# Change Log

## 0.1.1 - 2020-05-13
* Modifications of IP check
** removed license information from dependencies.md files
** removed comments from package.xml files and fixed license string
** removed stb_image.h from plotlabserver repository and created automatic download triggered by cmake

## 0.1.0 - 2020-04-15
* Initial Eclipse release
* New ROS nodes
  * adore_if_ros/adore_borderbird_node - simple visualization a simulated vehicle's local view
  * adore_if_ros/adore_feedbackcontroller_node - stabilization of vehicle around reference trajectories: linear feedback controller
  * adore_if_ros/adore_lfbehavior_node - lane-following-only planner (no lane changes)
  * adore_if_ros/adore_objectdetectionmodel_node - a simple (exemplary) sensor model for detection of other traffic participants in simulation
  * adore_if_ros/adore_timer_node - time signal for simulation, pause simulation
  * adore_if_ros/adore_vehiclemodel_node - simulation of a simple vehicle model (linear bicycle model)
  * sumo_if_ros/sumotraffic2ros_node - a node which synchronizes simulation in SUMO and ADORe vehicles in ROS
* New demos 
  * demo003: Simulation of an ADORe automated vehicle, which follows the lanes of an example map
  * demo004: Simulation of three ADORe automated vehicles on a circular track
  * demo005: Simulation of an ADORe automated vehicle and several SUMO vehicles on a circular track
  * demo006: Simulation of an ADORe automated vehicle, which navigates to a goal position by lane following and turning at junctions
  

## 0.0.1 - 2019-11-27
* Test release
* ROS interfaces for adore, ROS message definitions
* New ROS nodes
  * adore_mapprovider_node - loading of OpenDrive tracks for automated vehicle 
  * adore_navigation_node - computation of a shortest route to a given target on loaded track
* New demos
  * demo001: Loading of OpenDrive tracks
  * demo002: Computation of distance functions on track
