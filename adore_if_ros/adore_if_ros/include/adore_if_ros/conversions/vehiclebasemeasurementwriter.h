// *******************************************************************************
// * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
// * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
// *
// * This program and the accompanying materials are made available under the 
// * terms of the Eclipse Public License 2.0 which is available at
// * http://www.eclipse.org/legal/epl-2.0.
// *
// * SPDX-License-Identifier: EPL-2.0 
// *
// * Contributors: 
// *  Jonas Rieck
// ********************************************************************************


#include <adore/fun/vehiclebasemeasurement.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

namespace adore
{
namespace if_ROS
{
/**
 * ROS specific implementation of AWriter for adore::fun::VehicleBaseMeasurement.
 * Transmits adore::fun::VehicleBaseMeasurement by sending the following ROS messages:
 * measured steering wheel angle in rad:

    std_msgs::Float32
    /vehicleXY/VEH/steering_angle_measured


    wheel speeds (speed above ground in m/s), FL,RL,RR,FR (TODO)

    std_msgs::Float32MultiArray
    /vehicleXY/VEH/wheel_speed
    yaw-rate measured by ESP (rad/s) (TODO)

    std_msgs::Float32
    /vehicleXY/VEH/yaw_rate


    acceleration measured by ESP (m/s², ax, ay TODO, az TODO)

    std_msgs::Float32
    /vehicleXY/VEH/ax
 */
class VehicleBaseMeasurementWriter : public adore::mad::AWriter<adore::fun::VehicleBaseMeasurement>
{
private:
  ros::Publisher steering_angle_pub_;
  ros::Publisher wheel_speed_pub_;
  ros::Publisher yawrate_esp_pub_;
  ros::Publisher ax_esp_pub_;
  ros::Publisher ay_esp_pub_;

public:
  VehicleBaseMeasurementWriter(ros::NodeHandle* n, const std::string steering_topic,
                               const std::string wheel_speed_topic, const std::string yawrate_esp_topic,
                               const std::string ax_esp_topic, const std::string ay_esp_topic, int qsize)
  {
    steering_angle_pub_ = n->advertise<std_msgs::Float32>(steering_topic, qsize);
    wheel_speed_pub_ = n->advertise<std_msgs::Float32MultiArray>(wheel_speed_topic, qsize);
    yawrate_esp_pub_ = n->advertise<std_msgs::Float32>(yawrate_esp_topic, qsize);
    ax_esp_pub_ = n->advertise<std_msgs::Float32>(ax_esp_topic, qsize);
    ay_esp_pub_ = n->advertise<std_msgs::Float32>(ay_esp_topic, qsize);
  }
  /// canWriteMore indicates whether more data can be written
  virtual bool canWriteMore() const override
  {
    return true;
  }
  /// write sends out data value
  virtual void write(const adore::fun::VehicleBaseMeasurement& value) override
  {
    std_msgs::Float32 steering_msg;
    steering_msg.data = value.getSteeringAngle();
    steering_angle_pub_.publish(steering_msg);

    std_msgs::Float32MultiArray wheel_msg;
    wheel_msg.data.insert(wheel_msg.data.end(),value.getWheelSpeeds().begin(),value.getWheelSpeeds().end());
    wheel_speed_pub_.publish(wheel_msg);

    std_msgs::Float32 yaw_msg;
    yaw_msg.data = value.getYawRate();
    yawrate_esp_pub_.publish(yaw_msg);

    std_msgs::Float32 ax_msg;
    ax_msg.data = value.getEspAx();
    ax_esp_pub_.publish(ax_msg);

    std_msgs::Float32 ay_msg;
    ay_msg.data = value.getEspAy();
    ay_esp_pub_.publish(ay_msg);
  }
};
}  // namespace if_ROS
}  // namespace adore