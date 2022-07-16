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
#include <adore/env/borderbased/border.h>

namespace adore
{
namespace env
{
namespace BorderBased
{
/**
 * @brief A class which augments a vector of Border* with some sampling features
 */
class BorderSequence: public std::vector<Border*>
{
    public:


    /**
     * @brief returns the total lenght of all contained borders
     * @todo count the gaps, e.g. distances between previous end and next start coordinate
     */
    double getLength() const
    {
        double result = 0.0;
        for(auto it = begin();it!=end();it++)
        {
            result += (*it)->getLength();
        }
        return result;
    }

    /**
     * @brief step through border sequence and sample the values along the borders
     * @param sample_array array with values to sample
     * @param i0 first index in sample array
     * @param i1 last valid index in sample array
     * @param out function pointer for accepting sampling result, out: (int,double,double,double)->void
     * example lambda: [&X,&Y,&Z](int i,double x,double y,double z)mutable{X[i]=x;Y[i]=y;Z[i]=z;}
     * the input sample sequence has to be monotonous
     * @return true if every sample could be computed
     */
    template<typename TArray1,typename TArray2>
    bool sample(TArray1 sample_array,int i0,int i1,TArray2& outx,TArray2& outy,TArray2& outz) 
    {
        if(size()==0)return false;
        auto value_position = begin();
        double range_start = 0;
        double range_end = (*begin())->getLength();
        for(int i=i0;i<=i1;i++)
        {
            double sample = sample_array[i];
            while(range_end<sample)
            {
                value_position++;
                if(value_position==end())return false;
                range_start = range_end;
                range_end += (*value_position)->getLength();
            }
            if(sample<range_start)return false;
            if((*value_position)->m_path==nullptr)
            {
                double s = (sample-range_start)/(range_end-range_start);
                outx[i] = (*value_position)->m_id.m_first.m_X + s*((*value_position)->m_id.m_last.m_X - (*value_position)->m_id.m_first.m_X);
                outy[i] = (*value_position)->m_id.m_first.m_Y + s*((*value_position)->m_id.m_last.m_Y - (*value_position)->m_id.m_first.m_Y);
                outz[i] = (*value_position)->m_id.m_first.m_Z + s*((*value_position)->m_id.m_last.m_Z - (*value_position)->m_id.m_first.m_Z);
            }
            else
            {
                auto p = (*value_position)->m_path->f(sample-range_start);
                outx[i] = p(0);
                outy[i] = p(1);
                outz[i] = p(2);
            }
        }
        return true;
    }



    struct LineIterator
    {
        std::vector<Border*>::iterator bit_;
        int idx_;
        bool valid_;
        LineIterator(std::vector<Border*>::iterator bit,int idx,bool valid=true):bit_(bit),idx_(idx),valid_(valid){}

        static LineIterator begin(std::vector<Border*>* bs)
        {
            if(bs->size()==0)return LineIterator(bs->end(),0,false);
            return LineIterator(bs->begin(),0);
        }
        static LineIterator end(std::vector<Border*>* bs)
        {
            if(bs->size()==0)return LineIterator(bs->end(),0,false);
            auto it = std::next(bs->begin(),bs->size()-1);
            if((*it)->m_path==nullptr)return LineIterator(it,1);
            return LineIterator(it,(*it)->m_path->getData().nc()-1);
        }


        bool operator!=(const LineIterator& other)const
        {
            if(!valid_&&!other.valid_)return false;
            else return (*bit_)!=(*other.bit_) || idx_!=other.idx_;
        }
        void operator++(int)
        {
            if( (*bit_)->m_path==nullptr )
            {
                if(idx_==0)
                {
                    idx_++;
                }
                else 
                {
                    idx_=0;
                    bit_++;
                }
            }
            else
            {
                if((*bit_)->m_path->getData().nc()-1==idx_)
                {
                    bit_++;
                    idx_ = 0;
                }
                else
                {
                    idx_++;
                }            
            }
        }
        void getValue_nopath(double& x0,double& y0,double& z0,double& x1,double& y1, double& z1)
        {
            x0 = (*bit_)->m_id.m_first.m_X;
            y0 = (*bit_)->m_id.m_first.m_Y;
            z0 = (*bit_)->m_id.m_first.m_Z;
            x1 = (*bit_)->m_id.m_last.m_X;
            y1 = (*bit_)->m_id.m_last.m_Y;
            z1 = (*bit_)->m_id.m_last.m_Z;
        }
        void getValue_gap(double& x0,double& y0,double& z0,double& x1,double& y1, double& z1)
        {
            auto next = std::next(bit_);
            x0 = (*bit_)->m_id.m_last.m_X;
            y0 = (*bit_)->m_id.m_last.m_Y;
            z0 = (*bit_)->m_id.m_last.m_Z;
            x1 = (*next)->m_id.m_first.m_X;
            y1 = (*next)->m_id.m_first.m_Y;
            z1 = (*next)->m_id.m_first.m_Z;
        }
        void getValue_normal(double& x0,double& y0,double& z0,double& x1,double& y1, double& z1)
        {
            x0 = (*bit_)->m_path->getData()(1,idx_);
            y0 = (*bit_)->m_path->getData()(2,idx_);
            z0 = (*bit_)->m_path->getData()(3,idx_);
            x1 = (*bit_)->m_path->getData()(1,idx_+1);
            y1 = (*bit_)->m_path->getData()(2,idx_+1);
            z1 = (*bit_)->m_path->getData()(3,idx_+1);
        }
        void getValue(double& x0,double& y0,double& z0,double& x1,double& y1, double& z1)
        {
            if((*bit_)->m_path==nullptr)
            {
                if(idx_==0)
                {
                    getValue_nopath(x0,y0,z0,x1,y1,z1);
                }
                else
                {
                    getValue_gap(x0,y0,z0,x1,y1,z1);
                }
            }
            else
            {
                if(idx_==(*bit_)->m_path->getData().nc()-1)
                {
                    getValue_gap(x0,y0,z0,x1,y1,z1);
                }
                else
                {
                    getValue_normal(x0,y0,z0,x1,y1,z1);                    
                }
                
            }
        }
    };

    LineIterator beginLines(){return LineIterator::begin(this);}
    LineIterator endLines(){return LineIterator::end(this);}
};

}
}
}
