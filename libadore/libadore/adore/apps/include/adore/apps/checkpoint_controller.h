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
 *   Daniel Heß - Virtual traffic lights, switched according to test driver
 ********************************************************************************/
#pragma once
#include <adore/env/afactory.h>
#include <adore/fun/afactory.h>
#include <adore/mad/adoremath.h>
#include <adore/params/afactory.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <vector>
#include <string>

namespace adore
{
namespace apps
{
/**
 * @brief A set of virtual traffic lights are switched according to test driver input.
 * CheckpointController is provided with a list of coorindates. Virtual traffic lights are 
 * placed at these coordinates, initially with red-light phase active.
 * If a clearance button is pressed by the test driver, the nearest checkpoint traffic light
 * in the lane-following view is switched to green-light for a given amount of time.
 */
class CheckpointController
{
private:
    adore::mad::AReader<adore::fun::VehicleMotionState9d>* x_reader_;/**< compare vehicle position to checkpoint position */
    adore::mad::AReader<adore::fun::VehicleExtendedState>* xx_reader_;/**< get clearance button state*/
    adore::env::AFactory::TControlledConnectionWriter* cc_writer_;/**< write checkpoint state*/
    adore::env::ThreeLaneViewDecoupled lv_;/**<project checkpoint to lane*/
    std::vector<adore::env::ControlledConnection> checkpoints_;/**<set of checkpoints*/
    adore::params::APCheckpoints* pcheckpoints_;/**<parameters for checkpoints*/
public:
    CheckpointController(): lv_(false)
    {
        x_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleMotionStateReader();
        xx_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleExtendedStateReader();
        cc_writer_ = adore::env::EnvFactoryInstance::get()->getCheckPointWriter();
        pcheckpoints_ = adore::params::ParamsFactoryInstance::get()->getCheckpoints();
    }
    ~CheckpointController()
    {
        delete x_reader_;
        delete xx_reader_;
        delete cc_writer_;
        delete pcheckpoints_;
    }

    void run()
    {
        adore::fun::VehicleMotionState9d x;
        adore::fun::VehicleExtendedState xx;
        if(!x_reader_->hasData())return;
        if(!xx_reader_->hasData())return;
        x_reader_->getData(x);
        xx_reader_->getData(xx);
        lv_.update();

        double max_distance = pcheckpoints_->getClearDistance();
        double timeout = pcheckpoints_->getClearTimeout();
        bool button_pressed = xx.getCheckpointClearance();
        double t0 = x.getTime()+timeout;
        double t1 = x.getTime()+timeout + 24.0*60.0*60.0;

        if(!lv_.getCurrentLane()->isValid())return;
        double s_ego,n;
        lv_.getCurrentLane()->toRelativeCoordinates(x.getX(),x.getY(),s_ego,n);
        for(adore::env::ControlledConnection& con: checkpoints_)
        {
            bool checkpoint_in_view = true;
            bool checkpoint_in_clear_distance = true;
            double s_con;
            lv_.getCurrentLane()->toRelativeCoordinates(con.getID().getFrom().get<0>(),con.getID().getFrom().get<1>(),s_con,n);
            if(s_con<lv_.getCurrentLane()->getSMin()||s_con>=lv_.getCurrentLane()->getSMax())checkpoint_in_view = false;
            if(checkpoint_in_view && 
                (n>lv_.getCurrentLane()->getOffsetOfLeftBorder(s_con)
                ||lv_.getCurrentLane()->getOffsetOfRightBorder(s_con)>n))checkpoint_in_view = false;
            if(!checkpoint_in_view || s_con-s_ego>max_distance)checkpoint_in_clear_distance = false;
            if(checkpoint_in_view)
            {
                if(button_pressed && checkpoint_in_clear_distance)
                {
                    con.clear();
                    con.insertStateEvent(adore::env::ConnectionStateEvent(
                                                adore::env::ConnectionState::PERMISSIVE___MOVEMENT___ALLOWED,
                                                t0,true,true,t0,t0
                                            ));
                    con.insertStateEvent(adore::env::ConnectionStateEvent(
                                                adore::env::ConnectionState::STOP___AND___REMAIN,
                                                t1,false,false,t1,t1
                                            ));
                }
                else
                {
                    if(con.data_.size()==0)
                    {
                        con.insertStateEvent(adore::env::ConnectionStateEvent(
                                                    adore::env::ConnectionState::STOP___AND___REMAIN,
                                                    t1,false,false,t1,t1
                                            ));
                    }
                }                   
                cc_writer_->write(con);
            }
        }
    }

    /**
     * @brief read a set of 2d/3d coordinates from a csv file and add these to checkpoints_
     * @param filename the input csv file path/filename
     * The input text file should be structured as follows:
     * X0; Y0[; Z0][#comment]
     * X1; Y1[; Z1][#comment]
     * ....
     * E.g. two to three doubles per line, separated by semicolons, "[]" meaning optional.
     */
    bool loadFromFile(std::string filename)
    {
        std::string line;
        std::ifstream file(filename);
        if(!file.is_open())
        {
            std::cout<<"failed to open file: "<<filename<<std::endl;
            return false;
        }
        while(std::getline(file,line))
        {
            size_t p0,p1,p2,p3;
            double X = 0.0,Y = 0.0,Z = 0.0;

            p3 = line.find("#");
            if(p3==std::string::npos)p3 = line.length();
            p0 = line.find(";");
            if(p0==std::string::npos)
            {
                std::cout<<"error in file: "<<filename<<std::endl;
                return false;
            }
            else
            {
                std::stringstream ss0;
                ss0<<line.substr(0,p0);
                ss0>>X;

                p1 = line.find(";",p0+1);
                if(p1==std::string::npos)p1 = p3;           
                else p2 = p3;

                {
                    std::stringstream ss0;
                    ss0<<line.substr(p0+1,p1-1);
                    ss0>>Y;
                }

                if(p1<p3)
                {
                    std::stringstream ss0;
                    ss0<<line.substr(p1+1,p2-1);
                    ss0>>Z;
                }
            }
            checkpoints_.push_back(adore::env::ControlledConnection(X,Y,Z,X,Y,Z));
        }
        return true;
    }

};
}
}