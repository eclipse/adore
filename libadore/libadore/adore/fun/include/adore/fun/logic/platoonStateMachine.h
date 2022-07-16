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
using namespace  adore::fun;
namespace adore
{
    namespace fun
    {
        namespace logic
        {
        // class platoonStateMachine
        // {
        //     public:
            //NA : Not Able
            //WF : Want to From
            //J  : Joining
            //IP : In Platoon
            //L  : Leaving
            double tau;
            double tau_platooning;
            double tau_normalDriving; 
            enum PLATOONING_STATE {NotAble, WantToForm, Joining, InPlatoon, Leaving}; 
            enum AUTOMATION_LEVEL { SAE_LEVEL0, SAE_LEVEL1, SAE_LEVEL2, SAE_LEVEL3, SAE_LEVEL4, SAE_LEVEL5 };  
                    
            struct NA2WF : sc::event <NA2WF> {}; // "NOTABLE" to "WANTTOFORM"
            
            struct J2NA : sc::event <J2NA> {};  // "JOINING" to "NOTABLE"
            struct J2WF : sc::event <J2WF> {};  // "JOINING" to "WANTTOFORM"
            struct J2IP : sc::event <J2IP> {};  // "JOINING" to "IN_PLATOON"

            struct WF2J : sc::event <WF2J> {};  // "WANTTOFORM" to "JOINING"
            struct WF2NA : sc::event <WF2NA> {};  // "WANTTOFORM" to "NOTABLE"
            struct WF2IP : sc::event <WF2IP> {};  // "WANTTOFORM" to "IN_PLATOON"

            struct IP2J : sc::event <IP2J> {};  // "IN_PLATOON" to "JOINING"
            struct IP2WF : sc::event <IP2WF> {};  // "IN_PLATOON" to "WANT_TO_FORM"
            struct IP2NA : sc::event <IP2NA> {};  // "IN_PLATOON" to "NOT_ABLE"
            struct IP2L : sc::event <IP2L> {};  // "IN_PLATOON" to "LEAVING"

            struct L2WF : sc::event <L2WF> {};  // "LEAVING" to "WANT_TO_FORM"
            struct L2NA : sc::event <L2NA> {};  // "LEAVING" to "NOT_ABLE"

            struct NOT_ABLE;
            struct WANT_TO_FORM;
            struct JOINING;
            struct IN_PLATOON;
            struct LEAVING;
            int PlatooningState;

           
            struct PlatooningStateMachine: sc::state_machine <PlatooningStateMachine, WANT_TO_FORM> 
            {
                public:
                

                double getTimeHeadway() {return tau;}
                int getPlatooningState() {return PlatooningState;}
                double getInPlatoonTimeHeadway() {return 1.5;}


            };   ///WANT_TO_FORM is defined as an entry state of the state machine         
            struct WANT_TO_FORM : sc::simple_state<WANT_TO_FORM, PlatooningStateMachine>  //WANT_TO_FORM belongs to the state machine
            {
                WANT_TO_FORM () 
                { 
                    //std::cout<< "\nPSM TRANSITION TO WANT_TO_FORM";
                    tau = 1.5; //Platooning time headway
                    
                    PlatooningState = WantToForm;
                }
                typedef mpl::list<
                 sc:: transition<WF2NA, NOT_ABLE> ,
                 sc:: transition<WF2J, JOINING> ,
                 sc:: transition<WF2IP, IN_PLATOON> > reactions;
                 
            };
            struct NOT_ABLE: sc::simple_state <NOT_ABLE, PlatooningStateMachine>
            {
                NOT_ABLE () 
                { 
                   // std::cout<< "\nPSM TRANSITION TO NOT ABLE";
                    tau = 3.0; //not able time headway
                    PlatooningState = NotAble;
                }
                typedef sc:: transition<NA2WF, WANT_TO_FORM> reactions;
            };   
            struct JOINING: sc::simple_state <JOINING, PlatooningStateMachine>
            {
                JOINING () 
                { 
                    //std::cout<< "\nPSM TRANSITION TO JOINING";
                    tau = 1.5; //Platooning time headway
                    PlatooningState = Joining;
                }
                typedef mpl::list<
                 sc:: transition<J2WF, WANT_TO_FORM>,
                 sc:: transition<J2NA, NOT_ABLE>,
                 sc:: transition<J2IP, IN_PLATOON> > reactions;
            }; 
            struct IN_PLATOON: sc::simple_state <IN_PLATOON, PlatooningStateMachine>
            {
                IN_PLATOON () 
                { 
                    //std::cout<< "\nPSM TRANSITION TO IN_PLATOON";
                    tau = 1.5; //Platooning time headway
                    PlatooningState = InPlatoon;
                }
                typedef mpl::list<
                 sc:: transition<IP2J, JOINING> ,
                 sc:: transition<IP2WF, WANT_TO_FORM> ,
                 sc:: transition<IP2NA, NOT_ABLE> ,
                 sc:: transition<IP2L, LEAVING> >reactions;
            };  
            struct LEAVING: sc::simple_state <LEAVING, PlatooningStateMachine>
            {
                LEAVING () 
                { 
                    //std::cout<< "\nPSM TRANSITION TO EAVING";
                    tau = 3.; //Leaving time headway
                    PlatooningState = Leaving;
                }
                typedef mpl::list<
                 sc:: transition<L2WF, WANT_TO_FORM> ,
                 sc:: transition<L2NA, NOT_ABLE>> reactions;
            };                                                         

        //};
        }
    }
}