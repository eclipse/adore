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
 *    Daniel Heß - initial implementation and API
 ********************************************************************************/
#pragma once
#include <adore/mad/llinearpiecewisefunction.h>

namespace adore
{
namespace env
{
    class RampPrediction
    {
        public:
        void predict_s_tva(double s0,double v0,double a0,double delay,double a1,double vbound,double t0,double dt,double t1,adore::mad::LLinearPiecewiseFunctionM<double,3>& result)
        {
            predict_t_sva(s0,v0,a0,delay,a1,vbound,t0,dt,t1,result);
            int n = result.getData().nc();
            for(int i=0;i<n;i++)
            {
                double tmp = result.getData()(0,i);
                result.getData()(0,i)=result.getData()(1,i);
                result.getData()(1,i)=tmp;
                if(i>0 && result.getData()(0,i)<=result.getData()(0,i-1))//restore monotony
                {
                    result.getData()(0,i)=result.getData()(0,i-1)+0.00001;
                }
            }
        }
        void predict_s_tva(double s0,double v0,double a0,double vbound,double t0,double dt,double t1,adore::mad::LLinearPiecewiseFunctionM<double,3>& result)
        {
            predict_t_sva(s0,v0,a0,vbound,t0,dt,t1,result);
            int n = result.getData().nc();
            for(int i=0;i<n;i++)
            {
                double tmp = result.getData()(0,i);
                result.getData()(0,i)=result.getData()(1,i);
                result.getData()(1,i)=tmp;
                if(i>0 && result.getData()(0,i)<=result.getData()(0,i-1))//restore monotony
                {
                    result.getData()(0,i)=result.getData()(0,i-1)+0.00001;
                }
            }
        }
        void predict_t_sva(double s0,double v0,double a0,double vbound,double t0,double dt,double t1,adore::mad::LLinearPiecewiseFunctionM<double,3>& result)
        {
            int n = (std::ceil)((t1-t0)/dt);
            result.getData().set_size(3+1,n);
            double s = s0;
            double v = v0;
            double a = a0;
            double t = t0;
            for(int i=0;i<n;i++)
            {
                result.getData()(0,i)=t;
                result.getData()(1,i)=s;
                result.getData()(2,i)=v;
                result.getData()(3,i)=a;
                double dti = dt;
                if(t+dti>t1)dti=t1-t;
                t+=dti;
                if( a>0.0 && v+a*dti>vbound )
                {
                    a = 0.0;
                    v = vbound;
                }
                else if( a<0.0 && v+a*dti<0.0 )
                {
                    a = 0.0;
                    v = 0.0;
                }
                s+=(std::max)(0.0,v*dti+0.5*a*dti*dti);
                v+=a*dti;
            }
        }
        void predict_t_sva(double s0,double v0,double a0,double delay,double a1,double vbound,double t0,double dt,double t1,adore::mad::LLinearPiecewiseFunctionM<double,3>& result)
        {
            int n = (std::ceil)((t1-t0)/dt);
            result.getData().set_size(3+1,n);
            double s = s0;
            double v = v0;
            double a = delay>dt?a0:a1;
            double t = t0;
            for(int i=0;i<n;i++)
            {
 
                result.getData()(0,i)=t;
                result.getData()(1,i)=s;
                result.getData()(2,i)=v;
                result.getData()(3,i)=a;
                double dti = dt;
                if(t+dti>t1)dti=t1-t;
                t+=dti;
                a = (t+dti<t0+delay) ? a0 : a1;
                if( a>0.0 && v+a*dti>vbound )
                {
                    a = 0.0;
                    v = vbound;
                }
                else if( a<0.0 && v+a*dti<0.0 )
                {
                    a = 0.0;
                    v = 0.0;
                }
                s+=(std::max)(0.0,v*dti+0.5*a*dti*dti);
                v+=a*dti;
            }
        }
    };
}
}