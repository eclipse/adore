/********************************************************************************
 * Copyright (C) 2017-2023 German Aerospace Center (DLR).
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
#include <adore_if_ros/simfactory.h>
#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/paramsfactory.h>
#include <adore_if_ros/funfactory.h>

namespace adore
{
    namespace if_ROS
    {

        /**
         *  Base class for ros nodes to make factories available
         */
        class FactoryCollection
        {
        private:
            ros::NodeHandle *nh_;

        public:
            FactoryCollection(ros::NodeHandle *nh = nullptr)
            {
                nh_ = nh;
            }
            void init(ros::NodeHandle *nh = nullptr, std::string param_namespace = "")
            {

                if (nh != nullptr)
                {
                    nh_ = nh;
                }
                adore::env::EnvFactoryInstance::init(this->getFactory<ENV_Factory>());
                adore::fun::FunFactoryInstance::init(this->getFactory<FUN_Factory>());
                adore::sim::SimFactoryInstance::init(this->getFactory<SIM_Factory>());
                adore::params::ParamsFactoryInstance::init(this->getParamsFactory(param_namespace));
            }
            /**
             * getFactory - get Factory object
             *
             */
            template <typename T>
            T *getFactory()
            {
                if (nh_ == nullptr)
                {
                    throw std::runtime_error("No ros node handle available while calling getFactory()");
                }
                static T the_factory(nh_);
                return &the_factory;
            }
            PARAMS_Factory *getParamsFactory(std::string prefix = "")
            {
                if (nh_ == nullptr)
                {
                    throw std::runtime_error("No ros node handle available while calling getParamsFactory");
                }
                static PARAMS_Factory params_factory(*nh_, prefix);
                return &params_factory;
            }
        };
    } // namespace if_ROS
} // namespace adore