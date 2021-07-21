/********************************************************************************
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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>

struct Provider
{
    std::string prefix_;
    tf::TransformBroadcaster br_;
    ros::Subscriber subscriber_;
    bool first_set_;
    nav_msgs::Odometry first_;
    Provider()
    {
        ros::NodeHandle node;
        subscriber_ = node.subscribe("odom", 1, &Provider::receive, this);
        first_set_=false;
        prefix_ = "";//ros::this_node::getNamespace();
    }
    void receive(nav_msgs::OdometryConstPtr in)
    {
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(in->pose.pose.position.x,in->pose.pose.position.y, in->pose.pose.position.z) );
        tf::Quaternion q;
        q.setX(in->pose.pose.orientation.x);
        q.setY(in->pose.pose.orientation.y);
        q.setZ(in->pose.pose.orientation.z);
        q.setW(in->pose.pose.orientation.w);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, in->header.stamp, "UTMZone", prefix_+"baselink"));
        if(!first_set_)
        {
            first_ = *in;
            first_set_ = true;
            br_.sendTransform(tf::StampedTransform(transform, in->header.stamp, "UTMZone", "map"));            
        }        
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_transform_provider");
  Provider provider;
  ros::spin();
  return 0;
};

