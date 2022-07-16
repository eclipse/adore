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
#include <math.h>

namespace adore
{
	namespace mad
	{
		/**
		 * path_distance - computes the distance along a path by sampling the path derivative at fixed steps defined in x
		 * a matrix d is returned, which contains the cummulatitve sum (is therefore sorted)
		 * N is the dimension of the path - usually 2 for 2D paths, but could be 3D
		 * @param path Two dimensional path to be evaluated
		 * @param x Samples of path's domain
		 * @return Vector of cummulative distance along path at x samples
		 */
		template<typename T, int N>
		adoreMatrix<T, 1, 0>& path_distance(ALFunction<T, adoreMatrix<T, N, 1>>* path, const adoreMatrix<T, 1, 0>& x)
		{
			adoreMatrix<T, 1, 0> d;
			d = dlib::zeros_matrix<T>(1, x.nc());
			ALFunction<T, adoreMatrix<T, N, 1>>* v = path->create_derivative();
			for (int i = 1; i < x.nc(); i++)
			{
				adoreMatrix<T, N, 1> vi = v->f(x(i - 1));
				T n = 0;
				for (int j = 0; j < N; j++)
				{
					T nj = vi(j);
					n += nj*nj;
				}
				d(i) = d(i - 1) + sqrt(n);
			}
			delete v;
			return d;
		}

		/**
		 * create_trueDistancePath - creates a function object, which indexes the given path via true distance instead of its previous parameter.
		 * The function is created by computing the distance and chaining the inverse distance function with a clone of the path: d:x->s, => f: p(d^-1(s))
		 * N is the dimension of the path - usually 2 for 2D paths, but could be 3D
		 * count is the number of samples to take for the distance function
		 * @param path Two dimensional path to be evaluated
		 * @param count number of samples
		 * @return Function mapping from path domain to cummulative distance along path
		 */
		template<typename T, int N>
		ALFunction<T, adoreMatrix<T, 2, 1>>* create_trueDistancePath(ALFunction<T, adoreMatrix<T, N, 1>>* path, int count)
		{
			adoreMatrix<T, 1, 0> x;
			adoreMatrix<T, 1, 0> y;
			x = adore::linspace<T>(path->limitLo(), path->limitHi(), count);
			y = path_distance(path, x);
			return funop::chain(path->clone(), new LLinearPiecewiseFunctionS<T>(y, x));
		}
	}
}