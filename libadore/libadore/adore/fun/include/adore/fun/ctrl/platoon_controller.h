
/********************************************************************************

* Copyright (C) 2017-2020 German Aerospace Center (DLR)
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0
*

* Contributors:
* Abhishek Joshi {Reza Dariani}
*
* //Platoon controller suggested velocity for ego-vehicle.
********************************************************************************/

#pragma once
#include <adore/env/traffic/cooperativeusersprocess.h>
#include <adore/fun/nlpqlp_planner/planner_data.h>
#include <deque>

//TO DO Abhishek

//#include <PlatoonController_Feedback.h>

//#include <PlatoonController_h_inf.h>


namespace adore

{

    namespace fun

    {

        class PlatoonController

        {

 

        public:
        double error_preceding ;
        double error_leader;
        double control_preceding ;
        double control_leader;
        double currentTime;
        Planner_data data;

        //std::deque<double> plot_time;
       // static const int NumPlotPoints = 100;//



        //communication delay is assumed to be 1.

        PlatoonController(adore::env::CooperativeUsersProcess* cup, Planner_data data)

        {
           this->data = data;
           this->cup=cup;
           this->MAX_VELOCITY = data.MAX_VELOCITY;
           k1 = data.platoon_controller_kd;  //D preceding
           k2 = data.platoon_controller_kp;   //P preceding
           kI = data.platoon_controller_ki;
           k3 = 0.0001;  //D leader
           k4 = 0.002;  //P leader
           desiredTimeGap = 1.5;
           timeDifference = 0.1;
           pl = 1;
           pp = 1;
           previousErrorToLeader = 0.;
           previousErrorToPrecedor = 0.;
           error_I = 0.;
           suggested_velocity = MAX_VELOCITY;
        };


        ~PlatoonController() {}

        void update(adore::fun::VehicleMotionState9d &egoStates)
        {
          //double safetyMarginEgo = 1.0; //
          //double safetyMarginCU = 1.0;
          correction = 1.0;
          currentTime = egoStates.getTime();
          double vx = egoStates.getvx();
          vx = std::max(vx,3.);
          control_preceding = 0.;
          control_leader = 0.;
          error_preceding = 0.0;
          error_leader = 0.0;
          double preceding_v = MAX_VELOCITY;
          double leader_v = MAX_VELOCITY;
          suggested_velocity = MAX_VELOCITY;
          double preceding_s_behind_bumper ;
          double preceding_s_front_bumper ;
          double leader_s_behind_bumper ;
          double precedingPolicy_s;

          if(cup->precedingExists())   
          { 
             preceding_v = cup->getPreceding().MCM.currentTrajectory.v[0];
             preceding_s_behind_bumper = cup->getPreceding().progress_rear_side;
             preceding_s_front_bumper = cup->getPreceding().progress_front_side;
             precedingPolicy_s = (desiredTimeGap * vx) + cup->getEgoProgressFrontSide();
             error_preceding = preceding_s_behind_bumper - precedingPolicy_s;
             if(error_preceding<-0.5) {correction = 0.; error_I=0.0;}
             error_I += error_preceding*timeDifference;
             control_preceding = (error_preceding*k2) + ((error_preceding - previousErrorToPrecedor)/timeDifference)*k1 + error_I*kI*correction;
             previousErrorToPrecedor = error_preceding;
           
          }


         if(cup->leaderExists() &&  !cup->leaderIsPreceding())
          {
            //desiredTimeGapWithLeader = 3.5*desiredTimeGap;
            leader_v = cup->getLeader().MCM.currentTrajectory.v[0];
            leader_s_behind_bumper = cup->getLeader().progress_rear_side;
            double leaderPolicy_s = (desiredTimeGap * preceding_v) + (preceding_s_front_bumper);
            error_leader = leader_s_behind_bumper - leaderPolicy_s;
            control_leader = (error_leader * k4) + ((error_leader - previousErrorToLeader)/timeDifference)*k3;
            previousErrorToLeader = error_leader;
            //std::cout<<"\n: "<<leaderPolicy_s<<"\t"<< leader_s_behind_bumper<<"\t"<<error_leader;
           // std::cout<<"\n"<<error_leader<<"\t"<<control_leader;
          }
         
          suggested_velocity = preceding_v + control_preceding + control_leader;
          //std::cout<<"\n velocity from controller "<<suggested_velocity;
          if(suggested_velocity>= MAX_VELOCITY) suggested_velocity = MAX_VELOCITY;
          if(suggested_velocity < 0.2) suggested_velocity = 0;
          suggested_velocity = filter(suggested_velocity);
          if(data.print_platooning) std::cout<<"\n"<<error_preceding<<"\t"<<suggested_velocity;
        
        }


    double getVelocity() {return suggested_velocity;}
    double filter(double v)
    {
      double _v = std::floor(v);
      double rem = v - _v;
      rem = (std::floor(rem*10.))/10.;
      return _v + rem;

    }

       private:
       // adore::view::platoonView* p_view; 
       double correction;
        adore::env::CooperativeUsersProcess* cup;
        double MAX_VELOCITY;
        double kI;
        double kp;  // Preceding gap error controller
        double kl;  // Leader gap error controller
        double pl;  // Preceding car-following policy - can be set by user
        double pp;  // Leading car-following policy - can be set by user
        double desiredTimeGap;   //has to be set by user
        double desiredTimeGapWithLeader; // distance between ego and leader / velocity of the ego
        double timeDifference; //has to be set by user
        double k1,k2,k3,k4; //tuning knobs of kp (k1,k2) and kl (k3,k4) controllers
        double v1,v2,v3; // v1 = velocity from kp controller; v2 = target velocity of preceding vehicle; v3 = velocity of kl controller
        double velocity; //final velocity suggested by controllers
        double previousErrorToLeader;
        double currentErrorToLeader;
        double previousErrorToPrecedor;
        double currentErrorToPrecedor;
        double suggested_velocity; 
        double error_I;       

        };

    }

}
