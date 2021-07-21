<!--
*******************************************************************************
* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*  Daniel Heß
********************************************************************************
-->
### v2xsim is a ROS simulation package for ITSG5 radio messages 
For each simulated ITSG5 message X, a "SimX.msg" file is provided in msg folder.
A SimX message transports the original ROS version of the ITSG5 message and a meta information header of type "V2XMetaSIM",
which contains send power, send location, send time and the size of the message on the physical channgel as bytecount.
A node "channel_sim_node" is provided by this package: The node relays messages between the simulation channel "/SIM/v2x/X", (where "X" stands for message type) and the reception and send channels of a station, "v2x/incoming/X" and "v2x/outgoing/X".
```
/SIM/v2x/X: SimX.msg  -->  v2x/incoming/X: X.msg
/SIM/v2x/X: SimX.msg  <--  v2x/outgoing/X: X.msg
```
The channel_sim_node further receives its station's position information of type "nav_msgs/Odometry.msg" on topic "odom" in order to compare a messages send location and the station's location on reception of a message.

