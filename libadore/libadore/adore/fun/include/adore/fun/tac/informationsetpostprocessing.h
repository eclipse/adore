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

#pragma once
#include "anominalplannerinformation.h"
#include <adore/mad/llinearpiecewisefunction.h>

namespace adore
{
  namespace fun
  {
      /**
       * Usually, the constraints in the NominalPlannerInformationSet are pre-processed before trajectory optimization.
       * Nonetheless, sometimes it might be convenient to apply the NominalConstraints in post-processing, after optimization.
       * InformationSetPostProcessing applies NominalConstraints to finished plan and tests whether all constraints are satisfied.
       * The finished plan has to be supplied in road-relvative coordinates, as all NominalConstraints are thus formulated.
       * Template parameters direclty correspond to referenced NominalPlannerInformationSet.
       */
	  template<int N,int D>      
      class InformationSetPostProcessing
      {
        public:
            typedef NominalPlannerInformationSet<N+1,2> TInformationSet;
        private:
            TInformationSet info_;/**< the set of constraints, which are evaluated */  
        public://
            TInformationSet& getInformationSet()
            {
             return info_;
            }
            /**
             * Test the validity of a longitudinal plan. Returns true if valid.
             * @param K number of derivatives given by longitudinal plan
             * @param t0_offset time offset of plan
             * @param s0_offset longitudinal position offset of plan
             * @param longitudinal_plan trajectory t->(s,s',s''...), where s is the component on the first axis of the road coordinate system
             */
            template<int K>
            bool isLongitudinalPlanValid(double t0_offset, double s0_offset,
                         adore::mad::LLinearPiecewiseFunctionM<double,K>* longitudinal_plan)
            {
                double trel,t,s,ds;
                auto& data = longitudinal_plan->getData();
                for(int i=0;i<data.nc();i++)
                {
                    trel = data(0,i);
                    t = trel + t0_offset;
                    s = data(1,i) + s0_offset;
                    ds = data(2,i);
                    auto value = longitudinal_plan->f(trel);
                    value(0) = value(0) + s0_offset;
                    
                    for(int der = 0; der<K && der<N; der++)//loop over derivative order
                    {
                        if(!(info_.getLB(0,der,t,s,ds)<=value(der) && value(der)<=info_.getUB(0,der,t,s,ds)))return false;
                    }
                }
                return true;
            }
            /**
             * Test the validity of a lateral plan. Returns true if valid.
             * Test requires longitudinal and lateral plan for computation, but evaluates only lateral constraints.
             * @param K1 number of derivatives of longitudinal plan
             * @param K2 number of derivatives of lateral plan
             * @param t0_offset time offset of plan
             * @param s0_offset longitudinal position offset of plan
             * @param longitudinal_plan trajectory t->(s,s',s''...), where s is the component on the first axis of the road coordinate system
             * @param lateral_plan trajectory t->(n,n',n''...), where n is the component on the second axis of the road coordinate system
             */
            template<int K1,int K2>
            bool isLateralPlanValid(double t0_offset, double s0_offset,
                         adore::mad::LLinearPiecewiseFunctionM<double,K1>* longitudinal_plan,
                         adore::mad::LLinearPiecewiseFunctionM<double,K2>* lateral_plan)
            {
                double trel,t,s,ds;
                auto& data = longitudinal_plan->getData();
                for(int i=0;i<data.nc();i++)
                {
                    trel = data(0,i);
                    t = trel + t0_offset;
                    s = data(1,i) + s0_offset;
                    ds = data(2,i);
                    auto value = lateral_plan->f(trel);

                    for(int der = 0; der<K2 && der<N; der++)//loop over derivative order
                    {
                        if(!(info_.getLB(1,der,t,s,ds)<=value(der) && value(der)<=info_.getUB(1,der,t,s,ds)))return false;
                    }
                }
                return true;
            }
      };
  }
}