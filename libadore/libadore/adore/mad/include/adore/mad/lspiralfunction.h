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

#include <adore/mad/alfunction.h>
#include <adore/mad/lpolynomial.h>
#include <adore/mad/llinearpiecewisefunction.h>
#include <adore/mad/oderk4.h>
#include <adore/mad/adoremath.h>
#include <adore/mad/rotations.h>
#include <adore/mad/integratorchain.h>
#include <algorithm>
#define _USE_MATH_DEFINES //for cmath
#include <math.h>
#include <iostream>

namespace adore
{
	namespace mad
	{
		/**
		 * A two-dimensional spiral function of degree M. Spiral's curvature has M derivatives
		 */
		template<typename T, int M> //M is degree of spiral - 0 for circle, 1 for clothoid, etc.
		class LSpiralFunction : public AScalarToN<T, 2>// ALFunction<T,adoreMatrix<T,2,1>>, f: s->X,Y
		{
		public:
			LLinearPiecewiseFunctionM<T, 3 + M + 1>* m_linear_data;
			class diffspiral :public AOdeModel<T>
			{
			public:
				diffspiral() {}
				virtual void f(T t, const adoreMatrix<T, 0, 1>& x_in, adoreMatrix<T, 0, 1>& dx_out)
				{
					dlib::set_all_elements(dx_out, (T)0);
					dx_out(0) = (T)1;//ds
					dx_out(1) = (std::cos)(x_in(3));//dX
					dx_out(2) = (std::sin)(x_in(3));//dY
					for (int i = 0; i <= M; i++)dx_out(3 + i) = x_in(3 + i + 1);//integrator chain, d/ds kappa^(i)=kappa^(i+1)
					//dx_out(3+0) = dpsi, always1
					//dx_out(3+M+1) = 0 //the last one is always 0
				}
			};
			LSpiralFunction() {}
			LSpiralFunction(LLinearPiecewiseFunctionM<T, 3 + M + 1>* linear_data) :m_linear_data(linear_data) {}
			virtual ~LSpiralFunction()
			{
				delete m_linear_data;
			}
			/**
			 *  instantiate spiral using initial parameter vector p0=[psi0,kappa0,dkappa0,ddkappa0,....]
			 *	sample the spiral at fixed distance ds
			 */
			LSpiralFunction(T S, T X0, T Y0, T ds, const adoreMatrix<T, 1, M + 2>& p0)
			{
				adoreMatrix<T, 1, 0> time = adore::mad::sequence<T>(0, ds, S);
				OdeRK4<T> rk4;
				adoreMatrix<T, 3 + M + 2, 1> x0;
				x0(0) = 0;//s
				x0(1) = X0;
				x0(2) = Y0;
				for (int i = 0; i <= M + 1; i++)x0(3 + i) = p0(i);//psi0,kappa0,dkappa0,ddkappa0...
				diffspiral myspiral;
				m_linear_data = new LLinearPiecewiseFunctionM<T, 3 + M + 1>(rk4.solve(&myspiral, time, x0));
			}
			/**
			 *  instantiate spiral using initial parameter vector p0=[psi0,kappa0,dkappa0,ddkappa0,....]
			 *	compute step size based on kappa_max and max admissible error
			 */
			LSpiralFunction(T S, T X0, T Y0, T kappa_max, T emax, const adoreMatrix<T, 1, M + 2>& p0)
			{
				T ds;
				if (kappa_max == 0)
				{
					ds = S;
				}
				else
				{
					//chose a stepsize in order to control deviation between path and linear path approximation
					ds = acos((T)1 - emax*kappa_max) / (kappa_max*(T)M_PI);
				}
				adoreMatrix<T, 1, 0> time = adore::mad::sequence<T>(0, ds, S);
				OdeRK4<T> rk4;
				adoreMatrix<T, 3 + M + 2, 1> x0;
				x0(0) = 0;//s
				x0(1) = X0;
				x0(2) = Y0;
				for (int i = 0; i <= M + 1; i++)x0(3 + i) = p0(i);//psi0,kappa0,dkappa0,ddkappa0...
				diffspiral myspiral;
				m_linear_data = new LLinearPiecewiseFunctionM<T, 3 + M + 1>(rk4.solve(&myspiral, time, x0));
			}
			/**
			 * instantiate spiral of order 1
			 */
			LSpiralFunction(T S, T X0, T Y0, T PSI0, T kappa0, T kappa1, T emax)
			{
				assert(M == 1);
				T kappa_max = (std::max)((std::abs)(kappa0), (std::abs)(kappa1));//TODO improve min/max estimate, implement lagrange remainder for poly in bounds
				T ds;
				if (kappa_max == 0)
				{
					ds = S;
				}
				else
				{
					//chose a stepsize in order to control deviation between path and linear path approximation
					ds = acos((T)1 - emax*kappa_max) / (kappa_max*(T)M_PI);
				}
				adoreMatrix<T, 1, 0> time = adore::mad::sequence<T>(0, ds, S);
				OdeRK4<T> rk4;
				adoreMatrix<T, 3 + M + 2, 1> x0;
				x0(0) = 0;//s
				x0(1) = X0;
				x0(2) = Y0;
				x0(3) = PSI0;
				x0(4) = kappa0;
				x0(5) = (kappa1 - kappa0) / S;
				diffspiral myspiral;
				m_linear_data = new LLinearPiecewiseFunctionM<T, 3 + M + 1>(rk4.solve(&myspiral, time, x0));
			}

			/**
			 * returns pointer to heading function, which is M+1 times differentiable
			 * instead of maintaining one instance of heading, the heading function is created each time anew, as the heading may change according to rotate operations on spiral
			 */
			virtual  ALFunction<T, T> * create_heading()
			{
				adoreMatrix<T, M + 2, 1> ic = dlib::subm(m_linear_data->getData(), dlib::range(3, M + 4), dlib::range(0, 0));
				return new LPolynomialS<T, M + 1>(
					poly_parameter_for_initial_condition<T, M + 1>(ic),
					m_linear_data->limitLo(),
					m_linear_data->limitHi()
					);
			}
		public://from ALFunction
			virtual adoreMatrix<T, 2, 1> f(T x) const override
			{
				adoreMatrix<T, 2, 1> y;
				y(0) = fi(x, 0);
				y(1) = fi(x, 1);
				return y;
			}

			virtual T limitHi() const override
			{
				return m_linear_data->limitHi();
			}

			virtual T limitLo() const override
			{
				return m_linear_data->limitLo();
			}

			virtual void setLimits(T lo, T hi) override
			{
				m_linear_data->setLimits(lo, hi);
			}
			virtual ALFunction<T, adoreMatrix<T, 2, 1> >* clone() override
			{
				return new LSpiralFunction<T, M>((LLinearPiecewiseFunctionM<T, 3 + M + 1>*)m_linear_data->clone());
			}
			virtual ALFunction<T, adoreMatrix<T, 2, 1> >* create_derivative()override
			{
				adoreMatrix<T, 2, 1> v;
				v = 1.0, 0.0;
				return funop::rotate<T>(create_heading(), new LConstFun<T, adoreMatrix<T, 2, 1>>(v,limitLo(),limitHi()));
			}
			virtual void bound(const T& xmin, const T& xmax, adoreMatrix<T, 2, 1> & ymin, adoreMatrix<T, 2, 1> & ymax)override
			{
				adoreMatrix<T, 3 + M + 1, 1> lymin, lymax;
				m_linear_data->bound(xmin, xmax, lymin, lymax);
				ymin = rowm(lymin, dlib::range(0, 1));
				ymax = rowm(lymax, dlib::range(0, 1));
			}
		public://from AScalarToN
			virtual ALFunction<T, T>* dimension(int i) override
			{
				return m_linear_data->dimension(i);
			}
			virtual T fi(T x, int dim) const override
			{
				return m_linear_data->fi(x, dim);
			}
			virtual void multiply(adoreMatrix<T, 0, 0> A, int rowi, int rowj)
			{
				m_linear_data->multiply(A, rowi, rowj);
			}
			virtual void add(adoreMatrix<T, 0, 1> b, int rowi, int rowj)
			{
				m_linear_data->add(b, rowi, rowj);
			}
		public:

			//virtual SUBFUN* getCurvature()
			//{
			//	return m_linear_data->dimension(3);
			//}
			//virtual SUBFUN* getCurvatureDerivative(int i)
			//{
			//	assert(i<=M);
			//	return m_linear_data->dimension(3+i);
			//}
			///**
			// *  create a copy function which contains Nnew=M-many dimensions, indices specified after M
			// */
			//template<int Nnew>
			//LLinearPiecewiseFunctionM<T,Nnew>* clone(int first,...)
			//{
			//	va_list args;
			//	va_start(args,first);
			//	return m_linear_data->clone<Nnew>(first,args);
			//}
			//LLinearPiecewiseFunctionM<T,2>* cloneXY()
			//{
			//	return this->clone<2>(0,1);
			//}

			///**
			// *   createNormalOffset creates an s->X,Y function, which has an offset in normal direction of this spiral, where the length of the normal offset is defined by the parameter
			// *   create a function according to the pattern formerly known as "parallelTO"
			// */
			//AScalarToN<T,2>* createNormalOffsetFunction(ALFunction<T,T>* normal_distance)
			//{
			//	return funop::add(
			//					cloneXY(),
			//					funop::multiply(
			//						funop::chain(
			//							new RotationFunction<T,2,1>(),
			//							getOrientation()->clone()
			//							),
			//						funop::stack(
			//							new LConstFunS((T)0),
			//							normal_distance
			//						)
			//					)
			//				);
			//}
		};
	}
}