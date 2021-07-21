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
 *   Reza Dariani - initial API and implementation 
 ********************************************************************************/
#include <ros/ros.h>
#include <adore_if_ros/baseapp.h>
#include <adore_if_ros_msg/OccupancyCylinderPredictionSet.h>
#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>
#include <adore/mad/coordinateconversion.h>
#include <v2xsim/SimMCM.h>


namespace adore
{
    namespace if_ROS
    {
        class mcm_to_prediction  : public Baseapp
        {
            private:
            adore::mad::AReader<adore::fun::VehicleMotionState9d>* state_reader_;
            adore::fun::VehicleMotionState9d state_;
            ros::Subscriber MCMSubscriber_;
            ros::Publisher prediction_publisher;
            adore_if_ros_msg::OccupancyCylinder cylinder;
            adore_if_ros_msg::OccupancyCylinderPrediction prediction;
            adore_if_ros_msg::OccupancyCylinderPredictionSet predictioSetMsg;
            int utm_zone_;
            int generationDeltaTime;
            int mem_generationDeltaTime; //memory
            int dt_betweenMessages;
            int v2xStationID;
            bool ignoreOldMsg;
            public:

        void init(int argc, char **argv, double rate, std::string nodename)
        {
            v2xStationID = 0;  
            Baseapp::init(argc, argv, rate, nodename);
            Baseapp::initSim();     
            prediction_publisher = getRosNodeHandle()->advertise<adore_if_ros_msg::OccupancyCylinderPredictionSet>("ENV/Prediction/expected",1);
            std::function<void()> run_fcn = (std::bind(&mcm_to_prediction::run_func, this)); 
            adore::if_ROS::FUN_Factory fun_factory(getRosNodeHandle());
            state_reader_ = fun_factory.getVehicleMotionStateReader();
            Baseapp::addTimerCallback(run_fcn);
            dt_betweenMessages = 0;
        }
        void receive_mcm(mcm_transaid_mcm_transaid::MCM msg)
        {
            if(v2xStationID == msg.header.stationID.value) return; //not process ego MCM
            prediction.occupancy.clear();
            getParam("PARAMS/UTMZone", utm_zone_);
            getParam("v2xStationID", v2xStationID);
            if((state_reader_)!=0 && state_reader_->hasData() )
            {
                state_reader_->getData(state_);
                double t = state_.getTime();   //any better way to read time?
                double T = t;
                auto msg_vm = msg.maneuverCoordination.mcmParameters;
                generationDeltaTime = msg.maneuverCoordination.generationDeltaTime.value;
                dt_betweenMessages = generationDeltaTime - mem_generationDeltaTime;
                mem_generationDeltaTime = generationDeltaTime;
                // oldMsg(dt_betweenMessages,msg.header.stationID.value);
                // if (ignoreOldMsg) return;
                if(msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements.size()>0)
                {
                    double X, Y;
                    double lat_deg = (double)msg_vm.basicContainer.referencePosition.latitude.value*1.0e-7;
                    double lon_deg = (double)msg_vm.basicContainer.referencePosition.longitude.value*1.0e-7;
                    adore::mad::CoordinateConversion::LatLonToUTMXY(lat_deg,lon_deg,utm_zone_,X,Y);                  
                    prediction.trackingID = msg.header.stationID.value;  //TO BE CHECKED
                    prediction.confidence = 0.9; //TO BE CHECKED  msg_vm.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
                    for(int i=0; i<msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements.size();i++)
                    {             
                        double dx = ((double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].deltaXCm.value * 0.01);
                        double dy = ((double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].deltaYCm.value  * 0.01);
                        X += dx;
                        Y += dy;
                        cylinder.x = X;
                        cylinder.y = Y;
                        cylinder.t0 = t;
                        t += (double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].deltaTimeMs.value  * 0.001;
                        cylinder.t1 = t;
                        cylinder.z0 = 0;//min height
                        cylinder.z1 = 2;//max height
                        cylinder.rxy = 2;
                        prediction.occupancy.push_back(cylinder);
                    }
                predictioSetMsg.data.push_back(prediction);  
            }
        
        }
    }
    void oldMsg(int timeBetweenMessages,int stationID, int threshold=100)
    {
        ignoreOldMsg = false;
        if(timeBetweenMessages>threshold)
        {
            std::cout<<threshold<<"  message with ID "<<stationID<<" is ignored   "<<timeBetweenMessages<<"\n";
            ignoreOldMsg = true;
        }
    }

    virtual void run_func()
    {
        MCMSubscriber_= getRosNodeHandle()->subscribe<mcm_transaid_mcm_transaid::MCM>("v2x/incoming/MCM",1,&mcm_to_prediction::receive_mcm,this);
        prediction_publisher.publish(predictioSetMsg);   
    }



        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char** argv) 
{    
    adore::if_ROS::mcm_to_prediction mcm2prediction;
    mcm2prediction.init(argc, argv, 20., "mcm_to_prediction_node");
    ROS_INFO("mcm_to_prediction_node namespace is: %s", mcm2prediction.getRosNodeHandle()->getNamespace().c_str());
    mcm2prediction.run();
    return 0;
}