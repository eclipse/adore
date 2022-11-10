<!--
********************************************************************************
* Copyright (C) 2017-2022 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Matthias Nichting
********************************************************************************
-->

# ADORe interfacing with CARLA

[adore_if_carla](https://github.com/DLR-TS/adore_if_carla) provides a coupling of [adore_if_ros](/adore_if_ros) and [CARLA](https://github.com/carla-simulator/carla/) based on [carla-ros-bridge](https://github.com/carla-simulator/ros-bridge). It allows to use ADORe to control autonomous vehicles in CARLA. Details on prerequisites and building of adore_if_carla and instructions for getting started can be found within the adore_if_carla repository. The coupling is currently in an experimental state. 

## ROS nodes of adore_if_carla
When using adore_if_carla, the parameter ```PARAMS/adore_if_carla/carla_namespace``` needs to be set withing the namespace of the adore vehicle to get the matching namespace of the topics published and subscribed by the carla-ros-bridge.


### clock2simtime
This node transfers the time signal of the carla-ros-bridge to the simulation time used by ADORe. clock2simtime node:
- subscribes to topic ```/clock``` ```rosgraph_msgs/Clock```
- advertises topic ```/SIM/utc``` ```std_msgs/Float64```

### objects2adore
This nodes translates the traffic objects present in CARLA to a format usable by ADORe. The node should be started within the adore vehicle namespace.
objects2adore node:
- subscribes to topic ```/carla/[carla vehicle namespace]/objects``` message type: ```derived_object_msgs/ObjectArray```
- advertises topic ```[adore vehicle namespace]/traffic``` message type: ```adore_if_ros_msg/TrafficParticipantSet```

### vehiclestate2adore
This node reformats and publishes the messages output by the carla-ros-bridge to represent the vehicle state in order to be usable in ADORe. The node should be started within the adore vehicle namespace.
vehiclestate2adore node:
- subscribes to topic ```/carla/[carla vehicle namespace]/odometry``` message type: ```nav_msgs/Odometry```
- subscribes to topic ```/carla/[carla vehicle namespace]/vehicle_info``` message type: ```carla_msgs/CarlaEgoVehicleInfo```
- subscribes to topic ```/carla/[carla vehicle namespace]/vehicle_status``` message type: ```carla_msgs/CarlaEgoVehicleStatus```
- advertises topic ```[adore vehicle namespace]/odom``` message type: ```nav_msgs/Odometry```
- advertises topic ```[adore vehicle namespace]/localization``` message type: ```nav_msgs/Odometry```

### ackermanncommand2carla
This node translates the messages for controlling the vehicle's motion output by ADORe to a format that is supported by carla_ackermann_control of the carla-ros-bridge to allow the controlling of a vehicle in CARLA. The node should be started within the adore vehicle namespace.
ackermanncommand2carla node:
- subscribes to topic ```[adore vehicle namespace]/FUN/MotionCommand/acceleration``` message type: ```std_msgs/Float32```
- subscribes to topic ```[adore vehicle namespace]/odom``` message type: ```nav_msgs/Odometry```
- advertises topic ```/carla/[carla vehicle namespace]/ackermann_cmd``` message type: ```ackermann_msgs/AckermannDrive```

### plot_longitudinal_control_info
This is a node that may help to design a controller for the carla vehicle by plotting the current longitudinal acceleration, the current motion command for the longitudinal acceleration (output by the feedback controller), and the throttle command (output by the carla_ackermann_control). The node should be started within the adore vehicle namespace.
- subscribes to topic ```/carla/[carla vehicle namespace]/ackermann_cmd``` message type: ```ackermann_msgs/AckermannDrive```
- subscribes to topic ```[adore vehicle namespace]/FUN/MotionCommand/acceleration``` message type: ```std_msgs/Float32```
- subscribes to topic ```[adore vehicle namespace]/VEH/ax``` message type: ```std_msgs/Float32```
- subscribes to topic ```/carla/[carla vehicle namespace]/vehicle_control_cmd``` message type: ```carla_msgs/CarlaEgoVehicleControl```
- subscribes to topic ```[adore vehicle namespace]/odom``` message type: ```nav_msgs/Odometry```
