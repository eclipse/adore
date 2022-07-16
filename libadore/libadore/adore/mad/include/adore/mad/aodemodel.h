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

#include "adoremath.h"

namespace adore
{
	namespace mad
	{
		/**
		 * A model, represented by a ordinary differential equation. Template T is type, e.
		 * g. float or double, template n is the number of dimensions of the differential
		 * equation.
		 */
		template<typename T>
		class AOdeModel
		{
		public:
			AOdeModel() {
			}

			virtual ~AOdeModel() {
			}

			/**
			 * evaluation of derivative.
			 * @param t point of time at which derivative is computed
			 * @param x_in state at which derivative is computed
			 * @param dx_out derivative of state output
			 */
			virtual void f(T t, const adoreMatrix<T, 0, 1>& x_in, adoreMatrix<T, 0, 1>& dx_out) = 0;
		};
		/**
		 * A model, represented by a ordinary differential equation and nonlinear output equation. Template T is type, e.
		 * g. float or double, template n is the number of dimensions of the differential
		 * equation.
		 */
		template<typename T>
		class AOdeModelWithOutput
		{
		public:
			AOdeModelWithOutput() {
			}

			virtual ~AOdeModelWithOutput() {
			}
			/**
			 * evaluation of derivative and output.
			 * @param t point of time at which derivative is computed
			 * @param x_in state at which derivative is computed
			 * @param dx_out derivative of state, written by function
			 * @param y_out model output, written by function
			 */
			virtual void fh(T t, const adoreMatrix<T, 0, 1>& x_in, adoreMatrix<T, 0, 1>& dx_out, adoreMatrix<T, 0, 1>& y_out) = 0;
		};
	}
}