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
#include <vector>
#include <algorithm>

namespace adore
{
	namespace mad
	{
		/**
		* LPiecewiseFunction - a function with upper and lower limit, which consists of multiple elements of general type ALFunction
		* LPiecewiseFunction is intended as a container for multiple, different type subfunctions
		*/
		template<typename DT, typename CT>
		class LPiecewiseFunction : public ALFunction<DT, CT>
		{
		private:
			std::vector<ALFunction<DT, CT>*> m_data;
			mutable unsigned int m_searchIndex;
			bool m_deconstruct_pieces;
		public:
			LPiecewiseFunction() :m_searchIndex(0), m_deconstruct_pieces(true) {}
			LPiecewiseFunction(bool deconstruct_pieces) :m_searchIndex(0), m_deconstruct_pieces(deconstruct_pieces) {}
			void setDeconstructPieces(bool value)
			{
				m_deconstruct_pieces = value;
			}
			virtual ~LPiecewiseFunction()
			{
				if (m_deconstruct_pieces)
				{
					for (unsigned int i = 0; i < m_data.size(); i++)delete m_data.at(i);
				}
			}
			//binary search to find index of x, throws an exception on failure
			unsigned int findIndex(DT x) const
			{
				if (x > limitHi() || x < limitLo())
				{
					throw FunctionOutOfBoundsException();
				}
				unsigned int lower = 0;
				unsigned int upper = m_data.size() - 1;
				m_searchIndex = (std::min)(m_searchIndex, upper);
				ALFunction<DT, CT>* current = m_data.at(m_searchIndex);
				while (current->limitHi() < x || current->limitLo() > x)
				{
					if (lower == upper)throw FunctionIndexingError();
					if (current->limitHi() < x)//search higher
					{
						lower = m_searchIndex;
						unsigned int new_index = (std::ceil)((float)(m_searchIndex + upper) / 2.0f);
						if (new_index == m_searchIndex)throw FunctionIndexingError();
						m_searchIndex = new_index;
						current = m_data.at(m_searchIndex);
					}
					else//search lower
					{
						upper = m_searchIndex;
						unsigned int new_index = (std::floor)((float)(m_searchIndex + lower) / 2.0f);
						if (new_index == m_searchIndex)throw FunctionIndexingError();
						m_searchIndex = new_index;
						current = m_data.at(m_searchIndex);
					}
				}
				return m_searchIndex;
			}
			void appendHi(ALFunction<DT, CT>* newfun)
			{
				if (m_data.size() > 0)
				{
					if(newfun->limitLo()<limitHi())
					{
						std::cout<<"domain error: old function's high limit is "<< limitHi()<<", new functions low limit is "<<newfun->limitLo()<<"\n";
					}
					assert(newfun->limitLo() >= limitHi());///@TODO test for a certain precision / equality
					assert(newfun->limitHi() > newfun->limitLo());
				}
				m_data.push_back(newfun);
			}
			void appendHi_shifted(ALFunction<DT, CT>* newfun)
			{
				if (m_data.size() > 0)
				{
					DT delta = limitHi() - newfun->limitLo();
					appendHi(funop::stretch(newfun, newfun->limitLo() + delta, newfun->limitHi() + delta));
				}
				else
				{
					appendHi(newfun);
				}
			}
		public://methods inherited from ALFunction interface
			//function evaluation returns y of codomain type CT for a value x of domain type DT
			virtual CT f(DT x) const override
			{
				if (m_data.size() == 0)throw FunctionNotInitialized();
				return m_data.at(findIndex(x))->f(x);
			}
			virtual DT limitHi() const override
			{
				if (m_data.size() == 0)throw FunctionNotInitialized();
				return m_data.at(m_data.size() - 1)->limitHi();
			}
			virtual DT limitLo() const override
			{
				if (m_data.size() == 0)throw FunctionNotInitialized();
				return m_data.at(0)->limitLo();
			}
			//reduce or increase the limit of the function
			virtual void setLimits(DT lo, DT hi) override
			{
				if (m_data.size() == 0)throw FunctionNotInitialized();
				
				// increase limits
				if(limitLo() > lo)
				{
					m_data.at(0)->setLimits(lo, m_data.at(0)->limitHi());
				}
				if(limitHi() < hi)
				{
					m_data.at(m_data.size() - 1)->setLimits(m_data.at(m_data.size() - 1)->limitLo(), hi);
				}

				std::vector<int> indicesToRemove;
				// reduce: throw away functions that are out of bounds
				// iterate over elements
				for(unsigned int i=0; i < m_data.size(); i++)
				{
					// if new lo is greater than limitHi(), remove partial function
					if(m_data.at(i)->limitHi() <= lo)
					{
						indicesToRemove.push_back(i);
					}
					else
					{
						// if new hi is lower than limitLo(), remove partial function
						if(m_data.at(i)->limitLo() >= hi)
						{
							indicesToRemove.push_back(i);
						}
						// else: there is overlap in functions
						else
						{
							// if new lo is greater than limitLo(): set new limitLo
							if(m_data.at(i)->limitLo() < lo)
							{
								m_data.at(i)->setLimits(lo, m_data.at(i)->limitHi());
							}
							// if new hi is lower than limitHi(): set new limitHi
							if(m_data.at(i)->limitHi() > hi)
							{
								m_data.at(i)->setLimits(m_data.at(i)->limitLo(), hi);
							}
						}
					}
				}
				// erase backwards because otherwise indices change through vector
				for(int i = indicesToRemove.size()-1; i >= 0; i--)
				{
					m_data.erase(m_data.begin()+indicesToRemove.at(i));
				}
				//m_data.at(0)->setLimits(lo, m_data.at(0)->limitHi());
				//m_data.at(m_data.size() - 1)->setLimits(m_data.at(m_data.size() - 1)->limitLo(), hi);
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				LPiecewiseFunction<DT, CT>* derivative = new LPiecewiseFunction<DT, CT>();
				for (auto it = m_data.begin(); it != m_data.end(); it++)
				{
					derivative->appendHi((*it)->create_derivative());
				}
				return derivative;
			}
			virtual ALFunction<DT, CT>* clone()override
			{
				LPiecewiseFunction<DT, CT>* result = new LPiecewiseFunction<DT, CT>();
				for (auto it = m_data.begin(); it != m_data.end(); it++)
				{
					result->appendHi((*it)->clone());
				}
				return result;
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				int i0 = findIndex(xmin);
				int i1 = findIndex(xmax);
				if (i0 == i1)
				{
					m_data.at(i0)->bound(xmin, xmax, ymin, ymax);
				}
				else
				{
					CT ymin_tmp, ymax_tmp;
					m_data.at(i0)->bound(xmin, m_data.at(i0)->limitHi(), ymin, ymax);
					m_data.at(i1)->bound(m_data.at(i1)->limitLo(), xmax, ymin_tmp, ymax_tmp);
					ymin = adore::mad::min(ymin, ymin_tmp);
					ymax = adore::mad::max(ymax, ymax_tmp);
					for (int i = i0 + 1; i < i1; i++)
					{
						m_data.at(i)->bound(ymin_tmp, ymax_tmp);
						ymin = adore::mad::min(ymin, ymin_tmp);
						ymax = adore::mad::max(ymax, ymax_tmp);
					}
				}
			}
		};
		template<typename T>
		LPiecewiseFunction<T, T>* createPiecewiseLinear(const adoreMatrix<T, 2, 0>& xymatrix)
		{
			LPiecewiseFunction<T, T>* that = new LPiecewiseFunction<T, T>(true);
			for (int i = 0; i < xymatrix.nc() - 1; i++)
			{
				T x0 = xymatrix(0, i);
				T x1 = xymatrix(0, i + 1);
				T y0 = xymatrix(1, i);
				T y1 = xymatrix(1, i + 1);
				that->appendHi(new LLinearFunction<T, T>(x0, x1, y0, (y1 - y0) / (x1 - x0)));
			}
			return that;
		}
		template<typename T>
		LPiecewiseFunction<T, T>* createPiecewiseLinear(const adoreMatrix<T, 1, 0>& xvec, const adoreMatrix<T>& yvec, int row)
		{
			LPiecewiseFunction<T, T>* that = new LPiecewiseFunction<T, T>(true);
			for (int i = 0; i < xvec.nc() - 1; i++)
			{
				T x0 = xvec(0, i);
				T x1 = xvec(0, i + 1);
				T y0 = yvec(row, i);
				T y1 = yvec(row, i + 1);
				that->appendHi(new LLinearFunction<T, T>(x0, x1, y0, (y1 - y0) / (x1 - x0)));
			}
			return that;
		}
		template<typename T, int n>
		LPiecewiseFunction<T, adoreMatrix<T, n, 1>>* createPiecewiseLinear(const adoreMatrix<T, 1, 0>& xvec, const adoreMatrix<T, n, 0>& ymatrix)
		{
			LPiecewiseFunction<T, adoreMatrix<T, n, 1>>* that = new LPiecewiseFunction<T, adoreMatrix<T, n, 1>>(true);
			for (int i = 0; i < xvec.nc() - 1; i++)
			{
				T x0 = xvec(i);
				T x1 = xvec(i + 1);
				adoreMatrix<T, n, 1> y0 = dlib::colm(ymatrix, i);
				adoreMatrix<T, n, 1> y1 = dlib::colm(ymatrix, i+1);
				that->appendHi(new LLinearFunction<T, adoreMatrix<T, n, 1>>(x0, x1, y0, (y1 - y0) / (x1 - x0)));
			}
			return that;
		}
		template<typename T, int n>
		LPiecewiseFunction<T, adoreMatrix<T, n, 1>>* createPiecewiseLinear(const adoreMatrix<T, n + 1, 0>& xymatrix)
		{
			LPiecewiseFunction<T, adoreMatrix<T, n, 1>>* that = new LPiecewiseFunction<T, adoreMatrix<T, n, 1>>(true);
			for (int i = 0; i < xymatrix.nc() - 1; i++)
			{
				T x0 = xymatrix(0, i);
				T x1 = xymatrix(0, i + 1);
				adoreMatrix<T, n, 1> y0 = dlib::subm(xymatrix,dlib::range(1,xymatrix.nr()),dlib::range(i, i));
				adoreMatrix<T, n, 1> y1 = dlib::subm(xymatrix,dlib::range(1,xymatrix.nr()),dlib::range(i+1, i+1));
				that->appendHi(new LLinearFunction<T, adoreMatrix<T, n, 1>>(x0, x1, y0, (y1 - y0) / (x1 - x0)));
			}
			return that;
		}
		template<typename T, int n>
		void sample(LPiecewiseFunction<T, adoreMatrix<T, n, 1>>* fun, T* xvec, T* ydest, int row, int count)
		{
			for (int i = 0; i < count; i++)
			{
				ydest[i] = fun->f(xvec[i])(row);
			}
		}
	}
}