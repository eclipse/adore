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

#include "alfunction.h"
#include <math.h>

namespace adore
{
	namespace mad
	{
		/**
		 * compute curvature of a path
		 * @param path function mapping from path parameter to X and Y, the path for which curvature is computed
		 * @param samples vector of path parameters at which curvature is evaluated
		 * @return a function, which maps from path parameter to curvature
		 */
		template<typename T>
		ALFunction<T, T>* sample_curvature(ALFunction<T, adoreMatrix<T, 2, 1>>* path, adoreMatrix<T, 1, 0> samples)
		{
			ALFunction<T, adoreMatrix<T, 2, 1>>* dp = path->create_derivative();
			ALFunction<T, adoreMatrix<T, 2, 1>>* ddp = dp->create_derivative();
			adoreMatrix<T, 1, 0> values;
			values = dlib::zeros_matrix<T>(1, samples.nc());
			adoreMatrix<T, 2, 1> dpx, ddpx;
			T v;
			for (int i = 0; i < samples.nc(); i++)
			{
				dpx = dp->f(samples(i));
				ddpx = ddp->f(samples(i));
				v = sqrt(dpx(0)*dpx(0) + dpx(1)*dpx(1));
				values(i) = (dpx(0)*ddpx(1) - ddpx(0)*dpx(1)) / (v*v*v);
			}
			delete dp;
			delete ddp;
			return new LLinearPiecewiseFunctionS<T>(samples, values);
		}


		/**
		 * provides derivative of curvature for a path defined by dimension d1 and d2 of a function 
		 */
		template<int d1,int d2,typename T,int nd>
		class PartialCurvatureDerivative2d:public ALFunction<T,T>
		{
		public:
			typedef ALFunction<T, adoreMatrix<T, nd, 1>> TPath;
		private:
			TPath* p;
			TPath* dp;
			TPath* ddp;
			TPath* dddp;
			bool owner_of_dpddp;
		public:
			/**
			 * constructor: initialize with function containing path, compute partial derivatives of function.
			 */
			PartialCurvatureDerivative2d(TPath* p)
				:p(p)
			{
				dp = p->create_derivative();
				ddp = dp->create_derivative();
				dddp = ddp->create_derivative();
				owner_of_dpddp=true;
			}
			/**
			 * constructor: initialize with existing partial derivatives
			 */
			PartialCurvatureDerivative2d(TPath* dp, TPath* ddp, TPath* dddp)
				:p(0),dp(dp),ddp(ddp),dddp(dddp)
			{
				 owner_of_dpddp=false;
			}
			~PartialCurvatureDerivative2d()
			{
				if(owner_of_dpddp)
				{
					delete dp;
					delete ddp;
					delete dddp;
				}
			}
			virtual void setLimits(T lo, T hi)
			{
				/*
				 * Curvature2d retrieves limits from reference path
				 */
				throw FunctionNotImplemented();
			}
			virtual T limitHi()
			{
				return dp->limitHi();
			}
			virtual T limitLo()
			{
				return dp->limitLo();
			}
			/**
			 * returns derivative of the curvature of the path
			 */
			virtual T f(T x) const
			{
				auto dq = dp->f(x);
				auto ddq = dp->f(x);
				auto dddq = dp->f(x);
				return (dddq(d2)*dq(d1)-dddq(d1)*dq(d2)) - (ddq(d2)*dq(d1)-ddq(d1)*dq(d2))*(ddq(d1)*dq(d1)+ddq(d2)*dq(d2))/(dq(d1)*dq(d1)+dq(d2)*dq(d2));
			}
			virtual ALFunction<T, T>* clone()
			{
				if(owner_of_dpddp)
				{
					return new PartialCurvatureDerivative2d<T>(p);
				}
				else
				{
					return new PartialCurvatureDerivative2d<T>(dp,ddp,dddp);
				}
			}
			virtual ALFunction<T, T>* create_derivative()
			{
				throw FunctionNotImplemented();
			}
			virtual void bound(const T& xmin, const T& xmax, T& ymin, T& ymax)
			{
				//this function could be implemented
				throw FunctionNotImplemented();
			}
		};


		/**
		 * provides curvature of a path defined by dimensions d1, d2 of a function
		 */
		template<int d1,int d2,typename T,int nd>
		class Curvature2d:public ALFunction<T,T>
		{
		public:
			typedef ALFunction<T, adoreMatrix<T, nd, 1>> TPath;
		private:
			TPath* p;
			TPath* dp;
			TPath* ddp;
			bool owner_of_dpddp;
		public:
			/**
			 * constructor: create from path by computing partial derivatives
			 */
			Curvature2d(TPath* p)
				:p(p)
			{
				dp = p->create_derivative();
				ddp = dp->create_derivative();
				owner_of_dpddp=true;
			}
			/**
			 * constructor: create from existing derivatives
			 */
			Curvature2d(TPath* dp, TPath* ddp)
				:p(0),dp(dp),ddp(ddp)
			{
				 owner_of_dpddp=false;
			}
			~Curvature2d()
			{
				if(owner_of_dpddp)
				{
					delete dp;
					delete ddp;
				}
			}
			virtual void setLimits(T lo, T hi)
			{
				/*
				 * Curvature2d retrieves limits from reference path
				 */
				throw FunctionNotImplemented();
			}
			virtual T limitHi()
			{
				return dp->limitHi();
			}
			virtual T limitLo()
			{
				return dp->limitLo();
			}
			/**
			 * computes the derivative of a path
			 */
			virtual T f(T x) const
			{
				auto dq = dp->f(x);
				auto ddq = ddp->f(x);
				return (ddq(d2)*dq(d1)-ddq(d1)*dq(d2)) / sqrt(dq(d1)*dq(d1)+dq(d2)*dq(d2));
			}
			virtual ALFunction<T, T>* clone()
			{
				if(owner_of_dpddp)
				{
					return new Curvature2d<T>(p);
				}
				else
				{
					return new Curvature2d<T>(dp,ddp);
				}
			}
			virtual ALFunction<T, T>* create_derivative()
			{
				if( owner_of_dpddp ) return new PartialCurvatureDerivative2d(p);
				else throw FunctionNotInitialized();
			}
			virtual void bound(const T& xmin, const T& xmax, T& ymin, T& ymax)
			{
				//this function could be implemented
				throw FunctionNotImplemented();
			}
		};
	}
}