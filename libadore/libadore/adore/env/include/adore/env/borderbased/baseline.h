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
#include <vector>
#include <adore/env/borderbased/bordersequence.h>
#include <adore/mad/adoremath.h>
#include "csaps.h"
#include <adore/mad/linearfunctiontypedefs.h>

namespace adore
{
namespace env
{
namespace BorderBased
{
/**
 * @brief A local, non-linear, smooth road coordinate system generated from a sequence of borders
 */
class Baseline
{
    private:
    int nFitPoints_;                    /**< how many breaks the poly regression of the baseline has*/
    int nSamplesPoints_;                /**< at how many points the baseline poly is sampled for linearization*/
    double fit_smoothness_;             /**< smoothness parameter passed to poly regression*/
    double lookbehind_;
    double lookahead_;
    double min_length_;
    bool valid_;
    public:
    csaps::DoubleArray breaks_s_;       /**< poly breaks in poly domain*/
    csaps::DoubleArray breaks_x_;       /**< poly fit points in x */
    csaps::DoubleArray breaks_y_;       /**< poly fit points in y */
    csaps::DoubleArray breaks_z_;       /**< poly fit points in z */
    csaps::DoubleArray samples_s_;      /**< poly samples in poly domain*/
    csaps::DoubleArray samples_x_;      /**< poly x values at sampling points*/
    csaps::DoubleArray samples_dx_;     /**< poly dx/ds values at sampling points*/
    csaps::DoubleArray samples_ddx_;    /**< poly ddx/dss values at sampling points*/
    csaps::DoubleArray samples_dddx_;   /**< poly dddx/dsss values at sampling points*/
    csaps::DoubleArray samples_y_;      /**< poly y values at sampling points*/
    csaps::DoubleArray samples_dy_;     /**< poly dy/ds values at sampling points*/
    csaps::DoubleArray samples_ddy_;    /**< poly ddy/dss values at sampling points*/
    csaps::DoubleArray samples_dddy_;   /**< poly dddy/dss values at sampling points*/
    csaps::DoubleArray samples_z_;      /**< poly z values at sampling points*/
    csaps::DoubleArray samples_dz_;     /**< poly dz/ds values at sampling points*/
    csaps::DoubleArray samples_ddz_;    /**< poly ddz/dss values at sampling points*/
    csaps::DoubleArray samples_dddz_;   /**< poly dddz/dss values at sampling points*/
    csaps::DoubleArray base_L_;         /**< recomputed true length values (path integral along (xy)-poly-curve)*/
    csaps::DoubleArray base_nx_;        /**< normal vector of curve (x component)*/
    csaps::DoubleArray base_ny_;        /**< normal vector of curve (y component)*/
    csaps::DoubleArray base_kappa_;     /**< curvature of xy-curve*/
    csaps::DoubleArray base_dkappa_;    /**< curvature change over true length*/
    using function_type_xyz = adore::mad::function_type_xyz;
    using function_type2d = adore::mad::function_type2d;
    using function_type_scalar = adore::mad::function_type_scalar;
    function_type_xyz    position_fct_;     /**< function: s-coordinate -> euclidian coordinates for smoothed centerline*/
    function_type2d      normal_fct_;
    function_type_scalar curvature_fct_;
    function_type_scalar curvatureDerivative_fct_;

    public:
    Baseline()
    {
        nFitPoints_ = 100;
        nSamplesPoints_ = 200;
        fit_smoothness_ = 0.05;
        lookbehind_ = 100.0;
        lookahead_ = 100.0;
        min_length_ = 1.0;
        valid_ = false;
        resize();
    }
    void resize()
    {  
        breaks_s_.resize(nFitPoints_);
        breaks_x_.resize(nFitPoints_);
        breaks_y_.resize(nFitPoints_);
        breaks_z_.resize(nFitPoints_);
        samples_s_.resize(nSamplesPoints_);
        samples_x_.resize(nSamplesPoints_);
        samples_dx_.resize(nSamplesPoints_);
        samples_ddx_.resize(nSamplesPoints_);
        samples_dddx_.resize(nSamplesPoints_);
        samples_y_.resize(nSamplesPoints_);
        samples_dy_.resize(nSamplesPoints_);
        samples_ddy_.resize(nSamplesPoints_);
        samples_dddy_.resize(nSamplesPoints_);
        samples_z_.resize(nSamplesPoints_);
        samples_dz_.resize(nSamplesPoints_);
        samples_ddz_.resize(nSamplesPoints_);
        samples_dddz_.resize(nSamplesPoints_);
        base_L_.resize(nSamplesPoints_);
        base_nx_.resize(nSamplesPoints_);
        base_ny_.resize(nSamplesPoints_);
        base_kappa_.resize(nSamplesPoints_);
        base_dkappa_.resize(nSamplesPoints_);
        position_fct_.setData(dlib::zeros_matrix<double>(4, nSamplesPoints_));
        normal_fct_.setData(dlib::zeros_matrix<double>(3, nSamplesPoints_));
        curvature_fct_.setData(dlib::zeros_matrix<double>(2, nSamplesPoints_));
        curvatureDerivative_fct_.setData(dlib::zeros_matrix<double>(2, nSamplesPoints_));
    }
    void setSmoothness(double value){fit_smoothness_=value;}
    bool isValid()const{return valid_;}
    double getLookAhead()const{return lookahead_;}
    double getLookBehind()const{return lookbehind_;}
    void setLookAhead(double value){lookahead_ = value;}
    void setLookBehind(double value){lookbehind_ = value;}
    int getNSamplePoints()const{return nSamplesPoints_;}
    int getNFitPoints()const{return nFitPoints_;}
    /**
     * @brief compute fit for borderSequence
     * @param borderSequence the positions to fit against
     * @param soffset the length offset from the borderSequence start used for margin computations
     */
    void update(BorderSequence& borderSequence,double soffset)
    {
        valid_ = false;
        const double total_length = borderSequence.getLength();
        const double s0 = adore::mad::bound(0.0,soffset-lookbehind_,total_length);
        const double s1 = adore::mad::bound(s0,soffset+lookahead_,total_length);
        if(s1-s0<min_length_)return;
        adore::mad::linspace(s0,s1,breaks_s_,nFitPoints_);//poly breaks
        borderSequence.sample(breaks_s_,0,nFitPoints_-1,breaks_x_,breaks_y_,breaks_z_);
        double shift = -breaks_s_[0];//start the baseline function at 0
        for(int i=0;i<nFitPoints_;i++)breaks_s_[i] = breaks_s_[i] + shift;
        csaps::UnivariateCubicSmoothingSpline fx(breaks_s_,breaks_x_,fit_smoothness_);
        csaps::UnivariateCubicSmoothingSpline fy(breaks_s_,breaks_y_,fit_smoothness_);
        csaps::UnivariateCubicSmoothingSpline fz(breaks_s_,breaks_z_,fit_smoothness_);
        adore::mad::linspace(breaks_s_[0], breaks_s_[nFitPoints_-1], samples_s_, nSamplesPoints_);//samples between poly breaks
        std::tie(samples_x_,samples_dx_,samples_ddx_,samples_dddx_) = fx(samples_s_);
        std::tie(samples_y_,samples_dy_,samples_ddy_,samples_dddy_) = fy(samples_s_);
        std::tie(samples_z_,samples_dz_,samples_ddz_,samples_dddz_) = fz(samples_s_);
        if(!computeDerivatives())return;
        for(int i=0;i<nSamplesPoints_;i++)
        {  
            position_fct_.getData()(0,i) = base_L_[i];
            normal_fct_.getData()(0,i) = base_L_[i];
            curvature_fct_.getData()(0,i) = base_L_[i];
            curvatureDerivative_fct_.getData()(0,i) = base_L_[i];
            position_fct_.getData()(1,i) = samples_x_[i];
            position_fct_.getData()(2,i) = samples_y_[i];
            position_fct_.getData()(3,i) = samples_z_[i];
            normal_fct_.getData()(1,i) = base_nx_[i];
            normal_fct_.getData()(2,i) = base_ny_[i];
            curvature_fct_.getData()(1,i) = base_kappa_[i];
            curvatureDerivative_fct_.getData()(1,i) = base_dkappa_[i];
        }
        valid_ = true;
    }

    bool computeDerivatives()
    {
        base_L_[0] = 0.0;
        for(int i=0;i<nSamplesPoints_-1;i++)
        {
            const double dx = samples_x_[i+1]-samples_x_[i];
            const double dy = samples_y_[i+1]-samples_y_[i];
            const double dL = std::sqrt(dx*dx+dy*dy);
            base_L_[i+1] = base_L_[i] + dL;//use for x(s(L))
        }
        for(int i=0;i<nSamplesPoints_;i++)
        {
            const double dxds = samples_dx_[i];
            const double dyds = samples_dy_[i];
            const double ddxdss = samples_ddx_[i];
            const double ddydss = samples_ddy_[i];
            const double dddxdsss = samples_dddx_[i];
            const double dddydsss = samples_dddy_[i];
            const double v = std::sqrt(dxds*dxds+dyds*dyds);//v=dL/ds
            if(v<1.0e-10)return false;//there is a kink in the poly
            const double dvds = (ddxdss*dxds+ddydss*dyds) / v;
            const double nx = -dyds/v;
            const double ny =  dxds/v;
            const double kappa = (ddydss*dxds-ddxdss*dyds)/(v*v*v);
            const double dkappads = kappa * dvds/v + (dddydsss*dxds-dddxdsss*dyds)/(v*v*v);
            const double dkappadL = dkappads/v;        
            base_nx_[i] = nx;
            base_ny_[i] = ny;
            base_kappa_[i] = kappa;
            base_dkappa_[i] = dkappadL;            
        }
        return true;
    }

    /**
     * @brief define a function, which represents the offset from baseline to the neighboring function
     * @return the number of faults
     */
    int defineOffset(BorderSequence& borderSequence,function_type_scalar* offset,double maximum_offset=10.0)
    {
        if((int)offset->getData().nc()!=nSamplesPoints_)
        {
            offset->setData(dlib::zeros_matrix<double>(2, nSamplesPoints_));
        }
        bool allow_backwards_steps = false;//@TODO if allowing backwards steps: slower computation but able to handle curve center in lane
        double x0,y0,z0,x1,y1,z1,xa,xb,nx0,ny0,nx1,ny1;
        bool xa_inside,xb_inside;
        auto current_index = borderSequence.beginLines();
        auto end_index = borderSequence.endLines();
        int faults = 0;
        for(int i=0;i<nSamplesPoints_;i++)
        {
            bool hit = false;
            auto it = current_index;
            for(;!hit && it!=end_index;it++)
            {
                it.getValue(x0,y0,z0,x1,y1,z1);
                nx0 = samples_x_[i]-maximum_offset*base_nx_[i];
                ny0 = samples_y_[i]-maximum_offset*base_ny_[i];
                nx1 = samples_x_[i]+maximum_offset*base_nx_[i];
                ny1 = samples_y_[i]+maximum_offset*base_ny_[i];
                adore::mad::intersectLines(nx0,ny0,nx1,ny1,x0,y0,x1,y1,
                                            xa,xb,xa_inside,xb_inside);
                hit = xa_inside&&xb_inside;
                if(hit)break;
            }
            offset->getData()(0,i) = base_L_[i];
            if(hit)
            {
                if(!allow_backwards_steps)current_index = it;
                offset->getData()(1,i) = xa*2.0*maximum_offset-maximum_offset;
            }
            else
            {
                offset->getData()(1,i) = 0.0;
                faults++;
            }
            
        }
        return faults;
    }
};
}
}
}