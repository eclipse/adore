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
 *   Daniel Heß - unit tests for controlledconnection.h
 *********************************************************************************/
#pragma once
#include <adore/env/tcd/controlledconnection.h>
#include <vector>

namespace adore
{
namespace env
{
namespace test
{
    /**
     * A class required for unit test of ControlledConnectionSet
     */
    class TestConnectionFeed: public  adore::mad::AFeed<adore::env::ControlledConnection> 
    {
        public:
        typedef adore::env::ControlledConnection T;
        std::vector<T> data_;
        virtual bool hasNext() const override
        {
            return data_.size()>0;
        }
        virtual void getNext(T& value) override
        {   
            value = data_.back();
            data_.pop_back();
        }
        virtual void getLatest(T& value) override
        {
            //method is not important
            value = data_.back();
            data_.pop_back();
        }



    };
}
}
}