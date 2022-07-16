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
#include <adore/mad/llinearpiecewisefunction.h>
#include <list>
#include <vector>

namespace adore
{
	namespace mad
	{
		/**
		 *	sample_adaptive - error sensitive sampling of points of a function
		 *  provide function f, derivative df and a maximum error emax
		 *  the sampling error is defined as normal distance between the returned pw linear approximation and f
		 *  @param T is the numeric type, e.g. float or double
		 * 	@param N is the number of the functions' codomain dimensions 
		 *	@param f f is the function to be sampled
		 *  @param df df is the derivative of f
		 *  @param edes edes is the desired sampling error (tradeoff between number of points and precision)
		 *  @param emax is an upper bound on the maximum error, which has to be guaranteed by the sampling strategy
		 *  @param xmin is a lower bound on the stepsize
		 *  @param xmax is an upper bound on the stepsize
		 *	@param xstep is the current/starting stepsize, xstep must be greater than 0
		 */
		template<typename T, int N>
		LLinearPiecewiseFunctionM<T, N>* sample_adaptive(ALFunction<T, adoreMatrix<T, N, 1>>* f, ALFunction<T, adoreMatrix<T, N, 1>>* df, T edes, T emax, T xmin, T xmax, T xstep, T precision=(T)0.0001)
		{
			//LLinearPiecewiseFunctionM<T,N>* result = new LLinearPiecewiseFunctionM<T,N>();
			//result->getData().set_size(N+1,(std::max)((T)1,(std::ceil)((f->limitHi()-f->limitLo())/xstep)));
			std::vector<adoreMatrix<T, N + 1, 1>> buffer;
			adoreMatrix<T, N + 1, 1> element;
			T x0, x1, e, dedx;
			adoreMatrix<T, N, 1> y0, y1, dymin, dymax, ymin, ymax;
			if(f->limitHi()-f->limitLo()>precision)
			{
				x0 =(std::min)(f->limitLo()+precision,f->limitHi());
				y0 = f->f(x0);

				//first element
				element(0, 0) = x0;
				set_rowm(element, dlib::range(1, N)) = y0;
				buffer.push_back(element);

				dedx = edes / xstep;
				for (;;)
				{
					do
					{
						xstep = (xstep + edes / dedx) / (T)2;//estimate of required stepsize which hopefully yields the desired error: take mean of current and predicted to prevent overreactions
						xstep = (std::max)(xmin*(T)0.9999, (std::min)(xstep, xmax));//bound stepsize xmin<xstep<xmax
						//x1 = (std::min)(x0 + xstep, f->limitHi())-precision); // original
						x1 = (std::min)(x0 + xstep, f->limitHi()); //mark_ro: trying to fix problem with signals exactly at end of road
						y1 = f->f(x1);
						try
						{
							df->bound(x0, x1, dymin, dymax);
							e = (std::max)(norm2<T, N>(y1 - y0 - dymin*(x1 - x0)), norm2<T, N>(y1 - y0 - dymin*(x1 - x0))) / (T)2;// ey <= 0.5 * dx  * edydx
							dedx = e / xstep;//estimate of error for unit step size
						}catch(...)
						{
							e=emax;
							xstep=xmin;
						}
					} while (e > emax && xstep > xmin);
					x0 = x1;
					y0 = y1;
					element(0, 0) = x0;
					set_rowm(element, dlib::range(1, N)) = y0;
					buffer.push_back(element);
					if (x0 >= f->limitHi()-precision*(T)2)break;
				}
			}
			LLinearPiecewiseFunctionM<T, N>* result = new LLinearPiecewiseFunctionM<T, N>();
			result->getData().set_size(N + 1, buffer.size());
			int i = 0;
			for (auto it = buffer.begin(); it != buffer.end(); it++)
			{
				set_colm(result->getData(), i++) = *it;
			}
			return result;
		}
	}
}