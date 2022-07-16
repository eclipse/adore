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

namespace adore
{
	namespace mad
	{
		/**
		 *  A function which stacks derivatives in the codomain vector.
		 *	IntegratorChain assumes that a given function represents derivatives in its different dimensions,
		 *  e.g. m_data->fi(x,1) = d/dx m_data->fi(x,0)
		 *       m_data->fi(x,2) = d^2/dx^2 m_data->fi(x,0)
		 *       m_data->fi(x,3) = d^3/dx^3 m_data->fi(x,0)
		 *  ...
		 */
		template<typename T, int depth>
		class IntegratorChain :public ALFunction<T, T>
		{
		private:
			int m_level;
			AScalarToN<T, depth>* m_data;
			typedef T DT;
			typedef T CT;
		public:
			IntegratorChain(AScalarToN<T, depth>* data, int level)
			{
				m_data = data;
				m_level = level;
			}
			~IntegratorChain()
			{
				delete m_data;
			}
		public:
			//return m_level entry in data function
			virtual CT f(DT x) const override
			{
				if (m_level < depth)
				{
					return m_data->fi(x, m_level);
				}
				else
				{
					return (T)0;
				}
			}
			virtual DT limitHi() const override { return m_data->limitHi(); }
			virtual DT limitLo() const override { return m_data->limitLo(); }
			virtual void invertDirection()override { m_data->invertDirection(); }
			virtual void setLimits(DT lo, DT hi)override { m_data->setLimits(lo, hi); }
			virtual ALFunction<DT, CT>* clone()override { return new IntegratorChain((AScalarToN<T, depth>*)m_data->clone(), m_level); }
			virtual ALFunction<DT, CT>* create_derivative()override { return new IntegratorChain((AScalarToN<T, depth>*)m_data->clone(), m_level + 1); }
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				if (m_level < depth)
				{
					m_data->dimension(m_level)->bound(xmin, xmax, ymin, ymax);
				}
				else
				{
					ymin = 0;
					ymax = 0;
				}
			}
		};
	}
}