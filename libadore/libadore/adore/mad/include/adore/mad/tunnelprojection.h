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
#include <adore/mad/fun_essentials.h>

namespace adore
{
	namespace mad
	{
		/**
		 *	Computes a peculiar projection, "tunnel projection", which is useful for matching positions on lanes
		 *    @param tunnel_center: a function s |-> x_tun,y_tun, mapping centerline from distance along centerline to Cartesian
		 *    @param tunnel_left: a function s |-> d_left, distance from center to left tunnel wall
		 *    @param tunnel_right: a funciton s |-> d_right, distance from center to right tunnel wall
		 *    @param finput:  a function t |-> (x_in,y_in), input function, which has to be projected, mapping from a parameter t to Cartesian: finput represents center of swath
		 *    @param winput: width of the input swath
		 *    @return result: a function t |-> (s,overlaps,r_left,r_right), with s longitudinal distance along tunnel and r relative lateral distance wrt to d_left and d_right. r\in[-1,1] implies that it is contained in tunnel.
		 */
		template <typename T,int dtun_center,int dtun_left,int dtun_right,int dfin>
		LLinearPiecewiseFunctionM<T,4> getTunnelProjection(	LLinearPiecewiseFunctionM<T,dtun_center>* tunnel_center,
															LLinearPiecewiseFunctionM<T,dtun_left>* tunnel_left,
															LLinearPiecewiseFunctionM<T,dtun_right>* tunnel_right,
															LLinearPiecewiseFunctionM<T,dfin>* finput,
															T winput,
															int ixtun=1,int iytun=2,int idleft=1,int idright=1,int ixin=1,int iyin=2)
		{
			adoreMatrix<T> resultmat;
			resultmat.set_size(5,finput->getData().nc());
			
			T w2 = winput/(T)2;
			T n=(T)0;
			T s = tunnel_center->getClosestParameter( finput->getData()(ixin,0),finput->getData()(iyin,0),ixtun,iytun,n);
			T dleft = tunnel_left->fi(s,idleft-1);
			T dright = tunnel_right->fi(s,idright-1);
			resultmat(0,0) = finput->getData()(0,0);
			resultmat(1,0) = s;
			resultmat(2,0) = adore::mad::overlaps(dright,dleft,n-w2,n+w2)?(T)1:(T)0;
			resultmat(3,0) = (n+w2)/(n+w2>(T)0?dleft:dright);
			resultmat(4,0) = (n-w2)/(n-w2>(T)0?dleft:dright);

			for(int i=1;i<finput->getData().nc();i++)
			{
				s = tunnel_center->getClosestParameter_local(finput->getData()(ixin,i),finput->getData()(iyin,i),ixtun,iytun,s,&n);
				dleft = tunnel_left->fi(s,idleft-1);
				dright = tunnel_right->fi(s,idright-1);
				resultmat(0,i) = finput->getData()(0,i);
				resultmat(1,i) = s;
				resultmat(2,i) = adore::mad::overlaps(dright,dleft,n-w2,n+w2)?(T)1:(T)0;
				resultmat(3,i) = (n+w2)/(n+w2>(T)0?dleft:dright);
				resultmat(4,i) = (n-w2)/(n-w2>(T)0?dleft:dright);
			}

			return LLinearPiecewiseFunctionM<T,4>(resultmat);
		}
	}
}