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
 *   Matthias Nichting - initial API and implementation
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/sim/schedulernotificationmanager.h>
#include <adore/sim/afactory.h>
#include <adore_if_ros/simfactory.h>
#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/paramsfactory.h>
#include <adore_if_ros/funfactory.h>
// #include <adore/apps/common/allFactory.h>

namespace adore
{
    namespace if_ROS
    {

        /**
        *  Base class for ros nodes - Baseapp provides functions that can be used by derived ros nodes. It handles the communication with the scheduler node.
        */
        class Baseapp
        {
        public:
            Baseapp() {}

        private:
            inline static ros::NodeHandle *n_ = 0;
            std::vector<ros::Timer> timers_;                                  // functions that are periodically called
            inline static adore::sim::SchedulerNotificationManager *snm_ = 0; // object to coordinate communication with the scheduler
            bool useScheduler_;                                               // true, if scheduler is used
            double rate_;                                                     // main rate of calling the functions in timers_ are called

        public:
            /**
             * schedulerCallback - notifies scheduler of the new upper bound in time  
             *  
             */
            inline static void schedulerCallback(const ros::TimerEvent &e)
            {
                snm_->notifyScheduler(ros::Time::now().sec, ros::Time::now().nsec);
            }
            /**
             * init - initializes the ros node
             * 
             */
            void init(int argc, char **argv, double rate, std::string nodename)
            {
                rate_ = rate;
                ros::init(argc, argv, nodename);
                ros::NodeHandle *n = new ros::NodeHandle();
                n_ = n;
                n_->param("/use_scheduler", useScheduler_, false);
                {
                    ros::NodeHandle np("~");
                    np.getParam("rate", rate_);
                }
                if(rate==0.0)useScheduler_=false;
                adore::env::EnvFactoryInstance::init(this->getFactory<ENV_Factory>());
                adore::fun::FunFactoryInstance::init(this->getFactory<FUN_Factory>());
                adore::params::ParamsFactoryInstance::init(this->getParamsFactory(""));
            }
            /**
             * getFactory - get Factory object
             * 
             */
            template <typename T>
            T *getFactory()
            {
                static T the_factory(Baseapp::getRosNodeHandle());
                return &the_factory;
            }
            PARAMS_Factory *getParamsFactory(std::string prefix)
            {
                static PARAMS_Factory params_factory(*Baseapp::getRosNodeHandle(), prefix);
                return &params_factory;
            }
            /**
             * initSim - intilizes functionalites for simulation
             * 
             */
            void initSim()
            {
                // scheduler
                if (useScheduler_)
                {
                    snm_ = new adore::sim::SchedulerNotificationManager(this->getFactory<SIM_Factory>(), std::hash<std::string>{}(ros::this_node::getNamespace() + ros::this_node::getName()), (uint32_t)(1e9 / rate_));
                    timers_.push_back(n_->createTimer(ros::Duration(1 / rate_), schedulerCallback));
                }
            }
            /**
             * getRosNodeHandle - return ros::NodeHandle pointer
             */
            static ros::NodeHandle *getRosNodeHandle()
            {
                return n_;
            }
            virtual void resume()
            {
                snm_->resume();
            }
            virtual void pause()
            {
                snm_->pause();
            }
            virtual void run()
            {
                while (n_->ok())
                {
                    ros::spin();
                }
            }
            inline static void func(std::function<void()> &callback, const ros::TimerEvent &te)
            {
                callback();
            }
            /**
             * addTimerCallback - add a function that should be called periodically
             */
            virtual void addTimerCallback(std::function<void()> &callbackFcn, double rate_factor = 1.0)
            {
                timers_.push_back(n_->createTimer(ros::Duration(1 / rate_ / rate_factor), std::bind(&func, callbackFcn, std::placeholders::_1)));
            }

            /**
             * getParam - retrieve ros parameter
             */
            template <typename T>
            bool getParam(const std::string name, T &val)
            {
                return n_->getParam(name, val);
            }
        };
    } // namespace if_ROS
} // namespace adore
