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
#define _USE_MATH_DEFINES 
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <utils/traci/TraCIAPI.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <adore_if_ros_msg/TrafficParticipantSetSimulation.h>
#include <adore_if_ros_msg/TrafficParticipantSimulation.h>
#include <unordered_set>

struct Timer
{
    public:
    double tUTC_;
    Timer():tUTC_(0.0){}
    void receive(const std_msgs::Float64ConstPtr& msg)
    {
        tUTC_ = msg->data;
    }
};

struct ROSVehicleSet
{
    public:
    std::unordered_map<int,adore_if_ros_msg::TrafficParticipantSimulation> data_;///<- a mapping from vehicle id to latest message
    void receive(adore_if_ros_msg::TrafficParticipantSimulationConstPtr msg)
    {
        //data_.push_back(*msg);
        if(data_.find(msg->simulationID)==data_.end())
        {
            data_.emplace(msg->simulationID,*msg);
        }
        else
        {
            data_[msg->simulationID] = *msg;
        }
    }
};


int main(int argc, char** argv) 
{
    ros::init(argc,argv,"sumotraffic2ros_node");
    ros::NodeHandle n;
    ros::Rate r(100.0);//process update rate
    ros::Rate r2(100.0);//update rate for retry server connect
    Timer timer;
    ROSVehicleSet rosVehicleSet;
    ros::Publisher publisher =  n.advertise<adore_if_ros_msg::TrafficParticipantSetSimulation>("/SIM/traffic/agg",5);
    ros::Subscriber subscriber1 = n.subscribe<std_msgs::Float64>("/SIM/utc",1,&Timer::receive,&timer);
    ros::Subscriber subscriber2 = n.subscribe<adore_if_ros_msg::TrafficParticipantSimulationConstPtr>("/SIM/traffic",100,&ROSVehicleSet::receive,&rosVehicleSet);
    std::string sumo_rosveh_prefix = "rosvehicle";
    

    std::string host = "localhost";
    int port = 1337;
    TraCIAPI client;
    bool connected = false;
    while (n.ok() && !connected)
    {
        try
        {
            std::cout<<"trying to connect to "<<host<<":"<<port<<std::endl;
            client.connect(host, port);
            connected = true;
        }
        catch(...)
        {
        }
        r2.sleep();        
    }

    double tSUMO0 = (double)client.simulation.getTime() ;
    double tSUMO = tSUMO0;

    std::unordered_map<std::string,int> sumoid2int;
    std::unordered_set<int> sumo_known_ids;


    while (n.ok())
    { 
        // update ROS/adore simulation time
        ros::spinOnce();

        // synchronize SUMO:
        double delta_t = timer.tUTC_-tSUMO+tSUMO0;
        if( delta_t >= 0.01 )
        {
            client.simulationStep(timer.tUTC_+tSUMO0);
            double tSUMO_new = (double)client.simulation.getTime();

            if( tSUMO_new>tSUMO )
            {
                tSUMO = tSUMO_new;
                std::cout<<"tUTC: "<<timer.tUTC_<<", tSUMO: " << tSUMO <<", delta_t="<<delta_t<<std::endl;


                // get vehicles from sumo
                std::vector<std::string> idlist = client.vehicle.getIDList();
                if( idlist.size() > 0 )
                {
                    //message for set of traffic participants
                    adore_if_ros_msg::TrafficParticipantSetSimulation tpset;
                    tpset.simulator = "SUMO";
                    for(auto& id:idlist)
                    {
                        //ignore vehicles controlled by other simulator
                        if(id.find(sumo_rosveh_prefix)==std::string::npos)
                        {
                            //id translation
                            int intid = 0;
                            auto idtranslation = sumoid2int.find(id);
                            if( idtranslation==sumoid2int.end() )
                            {
                                intid = sumoid2int.size()+1000.0;
                                sumoid2int.emplace(id,intid);
                            }
                            else
                            {
                                intid = idtranslation->second;
                            }

                            //retrieve vehicle data
                            libsumo::TraCIPosition tracipos = client.vehicle.getPosition(id);            
                            double heading = M_PI*0.5 -client.vehicle.getAngle(id)/180.0*M_PI;
                            double v = client.vehicle.getSpeed(id);
                            std::string type = client.vehicle.getTypeID(id);
                            double L = client.vehicle.getLength(id);
                            double w = client.vehicle.getWidth(id);
                            double H = client.vehicle.getHeight(id);
                            int signals = client.vehicle.getSignals(id);///<- bit array! see TraciAPI:669, VehicleSignal
                            
                            //ros message for single traffic participant
                            adore_if_ros_msg::TrafficParticipantSimulation tp;
                            tp.simulationID = intid;
                            tp.data.time = tSUMO-tSUMO0;
                            tp.data.shape.type = 1;
                            tp.data.shape.dimensions.push_back(L);
                            tp.data.shape.dimensions.push_back(w);
                            tp.data.shape.dimensions.push_back(H);
                            tp.data.motion_state.pose.pose.position.x = tracipos.x;
                            tp.data.motion_state.pose.pose.position.y = tracipos.y;
                            tp.data.motion_state.pose.pose.position.z = tracipos.z;
                            tf2::Quaternion q;
                            q.setRPY(0.0,0.0,heading);       
                            tp.data.motion_state.pose.pose.orientation.x = q.getX();
                            tp.data.motion_state.pose.pose.orientation.y = q.getY();
                            tp.data.motion_state.pose.pose.orientation.z = q.getZ();
                            tp.data.motion_state.pose.pose.orientation.w = q.getW();
                            tp.data.motion_state.twist.twist.linear.x = v;
                            //add the traffic participant to the set
                            tpset.data.push_back(tp);
                        }
                    }
                    //write the message
                    publisher.publish(tpset);
            
                }

                //update sumo with new information from ros
                for(auto pair:rosVehicleSet.data_)
                {
                    auto msg = pair.second;
                    std::stringstream ss;
                    ss<<sumo_rosveh_prefix<<msg.simulationID;
                    std::string sumoid = ss.str();
                    if(sumo_known_ids.find(msg.simulationID)
                        ==sumo_known_ids.end())
                    {
                        sumo_known_ids.emplace(msg.simulationID);
                        client.vehicle.add(sumoid,"");
                        client.vehicle.setMaxSpeed(sumoid,100.0);
                    }
                    auto p = msg.data.motion_state.pose.pose;
                    tf2::Quaternion q;
                    q.setValue(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
                    tf2::Matrix3x3 m(q);
                    double roll,pitch,yaw;
                    m.getRPY(roll,pitch,yaw);
                    const double heading = (M_PI*0.5 - yaw)*180.0/M_PI;

                    client.vehicle.moveToXY(sumoid,"",0,
                        p.position.x,
                        p.position.y,
                        heading,
                        0);
                    client.vehicle.setSpeed(sumoid,msg.data.motion_state.twist.twist.linear.x);
                }
            }
        }

        //wait for next update
        r.sleep();
    }

    client.close();
}
