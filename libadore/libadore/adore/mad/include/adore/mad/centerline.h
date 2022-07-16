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
#include <adore/mad/adoremath.h>
#include <adore/mad/llinearpiecewisefunction.h>

namespace adore
{
	namespace mad
	{
		/**
		 *	computes the centerline between two point-lists of size N and M. the resulting point list has size N+M-1. N>=2 and M>=2 required.
		 *	left is of size dxN, right is of size dxM, center is of size dx(N+M-2), with d>=4, so that a matrix is ( (s0,x0,y0,z0,...)^T  (s1,x1,y1,z1,...)^T ... )
		 *  the center matrix must be sufficiently sized before computeCenterline. 
		 *  Redundant points are removed from centerline, so that the actual number of distinct points is returned.
		 */
		inline int computeCenterline(const adoreMatrix<double,0,0>& left,const adoreMatrix<double,0,0>& right,adoreMatrix<double,0,0>& center)
		{
			int N = left.nc();
			int M = right.nc();
			// K unused?
			// int K = N + M - 2;
			auto rxyz = dlib::range(1,3);
			static const double min_length = 1e-3;

			//points ordered a-b-c in direction of path
			adoreMatrix<double,3,1> La,Lb,Lc,Lac;//points on the left
			adoreMatrix<double,3,1> Ra,Rb,Rc,Rac;//points on the right
			adoreMatrix<double,3,1> Ca,Cc;//points in the center

			//init first points
			La = dlib::subm(left,rxyz,dlib::range(0,0));
			Ra = dlib::subm(right,rxyz,dlib::range(0,0));
			Ca = (La+Ra)*0.5;//average of first points
			Lc = La;
			Rc = Ra;
			center(0,0) = 0;
			dlib::set_subm(center,rxyz,dlib::range(0,0)) = Ca;

			//indices of c points
			int i_center = 1;
			int i_left = 1;
			int i_right = 1;

			//lengths
			double length_Lac=0,length_Rac=0,length_Cac;
			//progress
			double progress_RonL,progress_LonR;

			while(i_left<N || i_right<M)
			{
				//assign c point and progress while a==c
				while(length_Lac<min_length && i_left<N)
				{
					Lc = dlib::subm(left,rxyz,dlib::range(i_left,i_left));
					Lac = Lc-La;
					length_Lac = adore::mad::norm2<double,3>(Lac);
					Lac = Lac / length_Lac;
					i_left++;
				}
				while(length_Rac<min_length && i_right<M)
				{
					Rc = dlib::subm(right,rxyz,dlib::range(i_right,i_right));
					Rac = Rc-Ra;
					length_Rac = adore::mad::norm2<double,3>(Rac);
					Rac = Rac / length_Rac;
					i_right++;
				}
				if(length_Lac>min_length)
				{
					progress_RonL = dlib::trans(Lac)*(Rc-La) / length_Lac;
					progress_RonL = (std::max)(0.0,progress_RonL);
				}
				else
				{
					progress_RonL = 1.0;
				}
				if(length_Rac>min_length)
				{
					progress_LonR = dlib::trans(Rac)*(Lc-Ra) / length_Rac;
					progress_LonR = (std::max)(0.0,progress_LonR);
				}
				else
				{
					progress_LonR = 1.0;
				}
				if(	std::abs(progress_RonL-progress_LonR)<min_length
					|| (progress_LonR>=1.0 && progress_RonL>=1.0)
					|| i_left==N || i_right==M)//same progress -> advance both
				{
					La = Lc;
					length_Lac = 0;
					Ra = Rc;
					length_Rac = 0;
				}
				else
				{
					if(progress_RonL<progress_LonR)//less progress on right side --> intermediate point on left side
					{
						La = La + Lac * progress_RonL * length_Lac;
						length_Lac = (1.0-progress_RonL) * length_Lac;
						Ra = Rc;
						length_Rac = 0;
					}
					else //less progress on left side --> intermediate point on right side
					{
						Ra = Ra + Rac * progress_LonR * length_Rac;
						length_Rac = (1.0-progress_LonR) * length_Rac;
						La = Lc;
						length_Lac = 0;
					}
				}
				Cc = (La+Ra)*0.5;
				length_Cac = adore::mad::norm2<double,3>(Cc-Ca);
				if( length_Cac > min_length )
				{
					dlib::set_subm(center,rxyz,dlib::range(i_center,i_center)) = Cc;
					center(0,i_center) = center(0,i_center-1) + length_Cac;
					Ca = Cc;
					i_center ++;
				}
			}
			if(i_center<2)
			{
				//centerline computation is degenerate: at least 2 points should be returned
				La =  dlib::subm(left,rxyz,dlib::range(0,0));
				Lb =  dlib::subm(left,rxyz,dlib::range(N-1,N-1));
				Ra =  dlib::subm(right,rxyz,dlib::range(0,0));
				Rb =  dlib::subm(right,rxyz,dlib::range(M-1,M-1));
				dlib::set_subm(center,rxyz,dlib::range(0,0)) = (La+Ra)*0.5;
				dlib::set_subm(center,rxyz,dlib::range(1,1)) = (Lb+Rb)*0.5;
				i_center = 2;
			}
			return i_center;
		}

		inline int computeCenterline(const LLinearPiecewiseFunctionM<double,3>& left,const LLinearPiecewiseFunctionM<double,3>& right,LLinearPiecewiseFunctionM<double,3>& center)
		{
			adoreMatrix<double, 0, 0> center_data;
			center.getData().set_size(4, left.getData().nc() + right.getData().nc() - 2);
			center_data.set_size(4, left.getData().nc() + right.getData().nc() - 2);
			int K = adore::mad::computeCenterline(left.getData(), right.getData(), center_data);
			center.setData(dlib::colm(center_data, dlib::range(0, K - 1)));
			return K;
		}
	}
}