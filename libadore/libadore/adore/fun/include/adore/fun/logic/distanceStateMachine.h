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
 *   Reza Dariani- initial API and implementation
 ********************************************************************************/
#pragma once
#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/mpl/list.hpp>
#include <iostream>
namespace sc = boost::statechart;
namespace mpl = boost::mpl;
namespace adore
{
    namespace fun
    {
        namespace logic
        {        
            //GD : Gap Distance
            //ND : Normal Distance
            //PD : Platooning Distance
            struct GAP_DISTANCE;
            struct NORMAL_DISTANCE;
            struct PLATOONING_DISTANCE;
            struct GD2ND : sc::event <GD2ND> {}; // "GAP_DISTANCE" to "NORMAL_DISTANCE"
            struct ND2PD : sc::event <ND2PD> {}; // "NORMAL_DISTANCE" to "PLATOONING_DISTANCE"
            struct ND2GD : sc::event <ND2GD> {}; // "NORMAL_DISTANCE" to "GAP_DISTANCE"
            struct PD2ND : sc::event <PD2ND> {}; // "PLATOONING_DISTANCE" to "NORMAL_DISTANCE"
            struct PD2GD : sc::event <PD2GD> {}; // "PLATOONING_DISTANCE" to "GAP_DISTANCE"
            
            struct DistanceStateMachine: sc::state_machine <DistanceStateMachine, NORMAL_DISTANCE> {};   ///NORMAL_DISTANCE is defined as an entry state of the state machine         
            struct NORMAL_DISTANCE : sc::simple_state<NORMAL_DISTANCE, DistanceStateMachine>  
            {
                NORMAL_DISTANCE () { std::cout<< "NORMAL_DISTANCE"<<std::endl;}
                typedef mpl::list<
                sc:: transition<ND2PD, PLATOONING_DISTANCE> ,
                sc:: transition<ND2GD, GAP_DISTANCE>> reactions;
            };
            struct GAP_DISTANCE : sc::simple_state<GAP_DISTANCE, DistanceStateMachine>  
            {
                GAP_DISTANCE () { std::cout<< "GAP_DISTANCE"<<std::endl;}
            // typedef mpl::list<
            typedef sc:: transition<GD2ND, NORMAL_DISTANCE> reactions;
            };   
            struct PLATOONING_DISTANCE : sc::simple_state<PLATOONING_DISTANCE, DistanceStateMachine>  
            {
                PLATOONING_DISTANCE () { std::cout<< "PLATOONING_DISTANCE"<<std::endl;}
                typedef mpl::list<
                sc:: transition<PD2GD, GAP_DISTANCE> ,
                sc:: transition<PD2ND, NORMAL_DISTANCE>> reactions;
            };  
        }             

    }

}