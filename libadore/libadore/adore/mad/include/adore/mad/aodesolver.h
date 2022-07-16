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

#include "aodemodel.h"

namespace adore
{
	namespace mad
	{
		/**
		 * Interface for ode solvers
		 */
		template<typename T>
		class AOdeSolver
		{
		public:
			AOdeSolver() {
			}

			virtual ~AOdeSolver() {
			}

			/**
			 * solve ode
			 * @param model the differential equation
			 * @param time the time steps over which to integrate
			 * @param x0 initial state
			 * @return a matrix with model states at requested points of time, dimension #x0 x #time
			 */
			virtual adoreMatrix<T> solve(AOdeModel<T>* model, const adoreMatrix<double, 1, 0>& time, const adoreMatrix<double, 0, 1>& x0) = 0;

			/**
			 * solve ode and provide model output
			 * @param model the differential equation and output function
			 * @param time the time steps over which to integrate
			 * @param x0 initial state
			 * @return a matrix with model states at requested points of time, dimension #x0 x #time
			 * @param Y_out a matrix to be filled with model outputs for given points of time
			 */
			virtual adoreMatrix<T> solve_with_output(AOdeModelWithOutput<T>* model, const adoreMatrix<double, 1, 0>& time, const adoreMatrix<double, 0, 1>& x0, adoreMatrix<double>& Y_out) = 0;
		};
	}
}