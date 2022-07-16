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
// #include <adore/mad/intervalarithmetic.h>
#define _USE_MATH_DEFINES //
#include <math.h>
#include <algorithm>

namespace adore
{
	namespace mad
	{
		template<typename T, int N>
		adoreMatrix<T, N, N> rotationMatrix(T angle)
		{
			adoreMatrix<T, N, N> M;
			M = dlib::identity_matrix<T>(N);
			M(0, 0) = (std::cos)(angle);		M(0, 1) = -(std::sin)(angle);
			M(1, 0) = (std::sin)(angle);		M(1, 1) = (std::cos)(angle);
			return M;
		}

		/**
		 *   a rotation function: R(f(x))=[cos(f(x)),-sin(f(x));sin(f(x)),cos(f(x))]
		 */
		template<typename T>
		class RotationFunction : public ALFunction<T, adoreMatrix<T, 2, 2>>
		{
		private:
			ALFunction<T, T>* m_sub;
			/**
			 *   a rotation functions derivative: [0,-f(x);f(x),0]
			 */
			//template<typename T>
			class RotationFunctionPD : public ALFunction<T, adoreMatrix<T, 2, 2>>
			{
			private:
				ALFunction<T, T>* m_sub;
			public:
				typedef T DT;
				typedef adoreMatrix<T, 2, 2> CT;
				virtual CT f(DT x) const override
				{
					adoreMatrix<T, 2, 2> R;
					R = dlib::ones_matrix<T>(2, 2);
					R(0, 0) = 0; R(0, 1) = -m_sub->f(x);
					R(1, 0) = m_sub->f(x); R(1, 1) = 0;
					return R;
				}
				virtual DT limitHi() const override
				{
					return m_sub->limitHi();
				}
				virtual DT limitLo() const override
				{
					return m_sub->limitLo();
				}
				virtual void setLimits(DT lo, DT hi)override
				{
					m_sub->setLimits(lo, hi);
				}
				virtual ALFunction<T, adoreMatrix<T, 2, 2>>* clone() override
				{
					return new RotationFunctionPD(m_sub->clone());
				}
				virtual ALFunction<DT, CT>* create_derivative() override
				{
					return new RotationFunctionPD(m_sub->create_derivative());
				}
				virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
				{
					interval<T> sub_bound;
					m_sub->bound(xmin, xmax, sub_bound.lb, sub_bound.ub);
					ymin(0, 0) = 0; ymin(0, 1) = (-sub_bound).lb;
					ymin(1, 0) = sub_bound.lb; ymin(1, 1) = 0;
					ymax(0, 0) = 0; ymax(0, 1) = (-sub_bound).ub;
					ymax(1, 0) = sub_bound.ub; ymax(1, 1) = 0;
				}

				RotationFunctionPD(ALFunction<T, T>* sub) :m_sub(sub) {}
				virtual ~RotationFunctionPD()
				{
					delete m_sub;
				}
			};

		private:
		public:
			typedef T DT;
			typedef adoreMatrix<T, 2, 2> CT;
			virtual CT f(DT x) const override
			{
				adoreMatrix<T, 2, 2> R;
				R = dlib::ones_matrix<T>(2, 2);
				R(0, 0) = (std::cos)(m_sub->f(x)); R(0, 1) = -(std::sin)(m_sub->f(x));
				R(1, 0) = (std::sin)(m_sub->f(x)); R(1, 1) = (std::cos)(m_sub->f(x));
				return R;
			}
			virtual DT limitHi() const override
			{
				return m_sub->limitHi();
			}
			virtual DT limitLo() const override
			{
				return m_sub->limitLo();
			}
			virtual void setLimits(DT lo, DT hi)override
			{
				m_sub->setLimits(lo, hi);
			}
			virtual ALFunction<T, adoreMatrix<T, 2, 2>>* clone() override
			{
				return new RotationFunction<T>(m_sub->clone());
			}
			virtual ALFunction<DT, CT>* create_derivative() override
			{
				return funop::mmultiply<T,2,2,2>(clone(), new RotationFunctionPD(m_sub->create_derivative()));
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				interval<T> angle_bound, cos_bound, sin_bound;
				m_sub->bound(xmin, xmax, angle_bound.lb, angle_bound.ub);
				cos_bound = adore::mad::cos(angle_bound);
				sin_bound = adore::mad::sin(angle_bound);
				ymin(0, 0) = cos_bound.lb; ymin(0, 1) = (-sin_bound).lb;
				ymin(1, 0) = sin_bound.lb; ymin(1, 1) = cos_bound.lb;
				ymax(0, 0) = cos_bound.ub; ymax(0, 1) = (-sin_bound).ub;
				ymax(1, 0) = sin_bound.ub; ymax(1, 1) = cos_bound.ub;
			}

			RotationFunction(ALFunction<T, T>* sub) :m_sub(sub) {}
			virtual ~RotationFunction()
			{
				delete m_sub;
			}
		};

		namespace funop
		{
			/**
			 *	rotate a 2d vector function
			 */
			template<typename T>
			ALFunction<T, adoreMatrix<T, 2, 1>>* rotate(ALFunction<T, T>* angle, ALFunction<T, adoreMatrix<T, 2, 1>>* vector)
			{
				return funop::mmultiply<T, 2,2,1>(new  RotationFunction<T>(angle), vector);
			}
		}
	}
}