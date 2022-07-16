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

#include "aodesolver.h"
#include "adoremath.h"

namespace adore
{
	namespace mad
	{
		/**
		 * A simple ode solver according to classical method of Runge and Kutta
		 */
		template<typename T>
		class OdeRK4 : public AOdeSolver<T>
		{
		public:
			OdeRK4() {
			}

			virtual ~OdeRK4() {
			}

			virtual adoreMatrix<T> solve(AOdeModel<T>* model, const adoreMatrix<double, 1, 0>& time, const adoreMatrix<double, 0, 1>& x0) override
			{
				int n = x0.nr();
				int m = time.nc();
				adoreMatrix<T> result(n, m);
				adoreMatrix<T, 0, 1> k1(n, 1);
				adoreMatrix<T, 0, 1> k2(n, 1);
				adoreMatrix<T, 0, 1> k3(n, 1);
				adoreMatrix<T, 0, 1> k4(n, 1);
				result.set_size(n, m);
				set_colm(result, 0) = x0;
				for (int i = 1; i < m; i++)
				{
					T dt = time(i) - time(i - 1);
					model->f(time(i - 1), colm(result, i - 1), k1);
					model->f(time(i - 1) + dt / (T)2, colm(result, i - 1) + (dt / (T)2)*k1, k2);
					model->f(time(i - 1) + dt / (T)2, colm(result, i - 1) + (dt / (T)2)*k2, k3);
					model->f(time(i - 1) + dt, colm(result, i - 1) + dt*k3, k4);
					set_colm(result, i) = colm(result, i - 1) + dt / (T)6 * (k1 + k2*(T)2 + k3*(T)2 + k4);
				}
				return result;
			}
			virtual adoreMatrix<T> solve_with_output(AOdeModelWithOutput<T>* model, const adoreMatrix<double, 1, 0>& time, const adoreMatrix<double, 0, 1>& x0, adoreMatrix<double>& Y_out) override
			{
				int n = x0.nr();
				int m = time.nc();
				adoreMatrix<T> result(n, m);
				adoreMatrix<T, 0, 1> k1(n, 1);
				adoreMatrix<T, 0, 1> k2(n, 1);
				adoreMatrix<T, 0, 1> k3(n, 1);
				adoreMatrix<T, 0, 1> k4(n, 1);
				adoreMatrix<T, 0, 1> y(Y_out.nr(),1);
				result.set_size(n, m);
				set_colm(result, 0) = x0;
				for (int i = 1; i < m; i++)
				{
					T dt = time(i) - time(i - 1);
					model->fh(time(i - 1), colm(result, i - 1), k1, y);//only the output of time(i-1) is stored
					set_colm(Y_out, i - 1) = y;
					model->fh(time(i - 1) + dt / (T)2, colm(result, i - 1) + (dt / (T)2)*k1, k2,y);
					model->fh(time(i - 1) + dt / (T)2, colm(result, i - 1) + (dt / (T)2)*k2, k3,y);
					model->fh(time(i - 1) + dt, colm(result, i - 1) + (dt / (T)2)*k1, k4,y);
					set_colm(result, i) = colm(result, i - 1) + dt / (T)6 * (k1 + k2*(T)2 + k3*(T)2 + k4);
				}

				model->fh(time(m - 1), colm(result, m - 1), k1, y);//evaluate fh for last point to get output
				set_colm(Y_out, m - 1) = y;
				return result;

			}
		};
	}
}