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
#include <algorithm>

namespace adore
{
	namespace mad
	{
		/**
		 * provides heading for dimensions 0,1 of a function
		 */
		template<typename T>
		class LHeading :public ALFunction<T, T>
		{
		private:
			AScalarToN<T, 2>* m_df;
		public:
			typedef adoreMatrix<T, 2, 1> CTDF;
			typedef T DT;
			typedef T CT;
			LHeading(AScalarToN<T, 2>* df) :m_df(df) {}
			virtual ~LHeading() { delete m_df; }
			virtual void setLimits(DT lo, DT hi)override
			{
				m_df->setLimits(lo, hi);
			}
			virtual DT limitHi() const override
			{
				return m_df->limitHi();
			}
			virtual DT limitLo() const override
			{
				return m_df->limitLo();
			}
			virtual CT f(DT x) const override
			{
				CTDF dy = m_df->f(x);
				return (T)(std::atan2)(dy(1), dy(0));
			}
			virtual ALFunction<DT, CT>* clone()override
			{
				return new LHeading<T>((AScalarToN<T, 2>*)m_df->clone());
			}
			/**
			 * creates a function providing the derivative of the heading
			 */
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				//compute curvature / derivative of heading:
				// kappa = (ddY*dX - ddX*dY) / (dX�+dY�)
				AScalarToN<T, 2>* df = m_df;
				AScalarToN<T, 2>* ddf = (AScalarToN<T, 2>*)m_df->create_derivative();

				auto result = funop::multiply<T, T, T, T>(
					funop::add<T, T>(
						funop::multiply<T, T, T, T>(
							ddf->dimension(1)->clone(),
							df->dimension(0)->clone()
							),
						funop::minus<T, T, T, T>(
							funop::multiply<T, T, T, T>(
								ddf->dimension(0)->clone(),
								df->dimension(1)->clone()
								)
							)
						),
					funop::reciprocal<T>(
						funop::add<T,T>(
							funop::multiply<T, T, T, T>(
								df->dimension(0)->clone(),
								df->dimension(0)->clone()
							),
							funop::multiply<T, T, T, T>(
								df->dimension(1)->clone(),
								df->dimension(1)->clone()
							)
						)
						)
					);
				delete ddf;
				return result;
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)
			{
				CTDF ymin_tmp;
				CTDF ymax_tmp;
				m_df->bound(xmin, xmax, ymin_tmp, ymax_tmp);
				interval<T> iy = adore::mad::atan2(interval<T>(ymin_tmp(1), ymax_tmp(1)), interval<T>(ymin_tmp(0), ymax_tmp(0)));
				ymin = iy.lb;
				ymax = iy.ub;
			}
		};

		namespace funop
		{
			/**
			 *	computes the heading of arbitrary (differentiable) paths
			 */
			template<typename T>
			ALFunction<T, T>* heading(AScalarToN<T, 2>* df)
			{
				return new adore::mad::LHeading<T>(df);
			}
		}
	}
}