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
#include <adore/mad/adoremath.h>
#include <adore/mad/csvlog.h>
#include <vector>
#include <algorithm>
#include <iostream>

namespace adore
{
	namespace mad
	{
		/**
		 *	A scalar, linear, limited, piecewise function
		 */
		template<typename T>
		class LLinearPiecewiseFunctionS :public ALFunction<T, T>
		{
		private:
			adoreMatrix<T, 2, 0> m_data;
			typedef T CT;
			typedef T DT;
			mutable unsigned int m_searchIndex;
		public:
			LLinearPiecewiseFunctionS(const adoreMatrix<T, 2, 0> &data)
			{
				m_data = data;
				m_searchIndex = 0;
			}
			LLinearPiecewiseFunctionS(const adoreMatrix<T, 1, 0> &x, const adoreMatrix<T, 1, 0> &y)
			{
				assert(x.nc() == y.nc());
				m_data = dlib::zeros_matrix<T>(2, x.nc());
				set_rowm(m_data, 0) = x;
				set_rowm(m_data, 1) = y;
				m_searchIndex = 0;
			}
			//binary search to find index of x, throws an exception on failure
			unsigned int findIndex(DT x) const
			{
				if (x > limitHi() || x < limitLo())
				{
					throw FunctionOutOfBoundsException();
				}
				unsigned int lower = 0;
				unsigned int upper = m_data.nc() - 2;
				m_searchIndex = (std::min)(m_searchIndex, upper);
				while (m_data(0, m_searchIndex + 1) < x || m_data(0, m_searchIndex) > x)
				{
					if (lower == upper)throw FunctionIndexingError();
					if (m_data(0, m_searchIndex + 1) < x)//upperLimit<x --> search higher
					{
						lower = m_searchIndex;
						unsigned int new_index = (std::ceil)((float)(m_searchIndex + upper) / 2.0f);
						if (new_index == m_searchIndex)throw FunctionIndexingError();
						m_searchIndex = new_index;
					}
					else//lowerLimit>x --> search lower
					{
						upper = m_searchIndex;
						unsigned int new_index = (std::floor)((float)(m_searchIndex + lower) / 2.0f);
						if (new_index == m_searchIndex)throw FunctionIndexingError();
						m_searchIndex = new_index;
					}
				}
				return m_searchIndex;
			}

			virtual CT f(DT x) const override
			{
				if (m_data.nc() == 0)throw FunctionNotInitialized();
				int i = findIndex(x);
				CT y0 = m_data(1, i);
				CT y1 = m_data(1, i + 1);
				return y0 + (y1 - y0)*((x - m_data(0, i)) / (m_data(0, i + 1) - m_data(0, i)));
			}
			virtual DT limitHi() const override
			{
				return m_data(0, m_data.nc() - 1);
			}
			virtual DT limitLo() const override
			{
				return m_data(0, 0);
			}
			virtual void setLimits(DT lo, DT hi)
			{
				throw FunctionNotImplemented();
			}
			virtual ALFunction<DT, CT>* clone()
			{
				return new LLinearPiecewiseFunctionS<T>(m_data);
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				throw FunctionNotImplemented();
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax) override
			{
				CT y;

				int imax = findIndex(xmax);
				y = f(xmax);
				ymin = y;
				ymax = y;

				int imin = findIndex(xmin);
				y = f(xmin);
				ymin = adore::mad::min(ymin, y);
				ymax = adore::mad::max(ymax, y);

				for (int i = imin + 1; i <= imax; i++)
				{
					y = m_data(i);
					ymin = adore::mad::min(ymin, y);
					ymax = adore::mad::max(ymax, y);
				}
			}
		};

		/**
		* LPiecewiseFunction - a linear interpolation function based on a matrix object, f: DT^1 --> T^n
		*/
		template<typename T, int n>
		class LLinearPiecewiseFunctionM : public AScalarToN<T, n>
		{
		private:
			adoreMatrix<T, n + 1, 0> m_data;
			mutable unsigned int m_searchIndex;
		public:
			typedef adoreMatrix<T, n, 1> CT;
			typedef typename AScalarToN<T,n>::DT DT;

			adoreMatrix<T, n + 1, 0>& getData()
			{
				return m_data;
			}

			const adoreMatrix<T, n + 1, 0>& getData()const
			{
				return m_data;
			}

			//binary search to find index of x, throws an exception on failure
			unsigned int findIndex(DT x, DT precision = 0.001) const
			{
				if (x > limitHi() || x < limitLo())
				{
					if(x > limitHi())
					{
						if(std::abs(x-limitHi())>precision)
						{
							std::cerr<<"LLinearPiecewiseFunctionM out of bounds: findIndex("<<x<<") called, while limitLo()="<<limitLo()<<"and limtiHi()="<<limitHi()<<std::endl;
							throw FunctionOutOfBoundsException();
						}
						else
						{
							x = limitHi();
						}
					}
					else
					{
						if(std::abs(x-limitLo())>precision)
						{
							std::cerr<<"LLinearPiecewiseFunctionM out of bounds: findIndex("<<x<<") called, while limitLo()="<<limitLo()<<"and limtiHi()="<<limitHi()<<std::endl;
							throw FunctionOutOfBoundsException();
						}
						else
						{
							x = limitLo();
						}
					}
				}
				unsigned int lower = 0;
				unsigned int upper = m_data.nc() - 2;
				m_searchIndex = (std::min)(m_searchIndex, upper);

				//look up likely places: last index, next index, previous index, first element, last element
				if (m_data(0, m_searchIndex) <= x)
				{
					if (x < m_data(0, m_searchIndex + 1))
					{
						return m_searchIndex;//the same as last time
					}
					else
					{
						if (m_searchIndex + 2 < (unsigned int)m_data.nc() && x < m_data(0, m_searchIndex + 2))
						{
							return ++m_searchIndex;//one to the right
						}
						else
						{
							if (m_data(0, m_data.nc() - 2) <= x)
							{
								return (m_searchIndex = m_data.nc() - 2); //the global last
							}
						}
					}
				}
				else
				{
					if (m_searchIndex - 1 >= 0 && m_data(0, m_searchIndex - 1) <= x)
					{
						return --m_searchIndex;//one to the left
					}
					else
					{
						if (x < m_data(0, 1))
						{
							return (m_searchIndex = 0);//the global first
						}
					}
				}

				//binary search
				while (m_data(0, m_searchIndex + 1) < x || m_data(0, m_searchIndex) > x)
				{
					if (lower == upper)throw FunctionIndexingError();
					if (m_data(0, m_searchIndex + 1) < x)//upperLimit<x --> search higher
					{
						lower = m_searchIndex;
						unsigned int new_index = (std::ceil)((float)(m_searchIndex + upper) / 2.0f);
						if (new_index == m_searchIndex)throw FunctionIndexingError();
						m_searchIndex = new_index;
					}
					else//lowerLimit>x --> search lower
					{
						upper = m_searchIndex;
						unsigned int new_index = (std::floor)((float)(m_searchIndex + lower) / 2.0f);
						if (new_index == m_searchIndex)throw FunctionIndexingError();
						m_searchIndex = new_index;
					}
				}
				return m_searchIndex;
			}
		public://methods inherited from ALFunction interface
			//function evaluation returns y of codomain type CT for a value x of domain type DT
			virtual CT f(DT x) const override
			{
				if (m_data.nc() == 0)throw FunctionNotInitialized();
				int i = findIndex(x);
				CT y0 = dlib::subm(m_data, dlib::range(1, n), dlib::range(i, i));
				CT y1 = dlib::subm(m_data, dlib::range(1, n), dlib::range(i + 1, i + 1));
				return y0 + (y1 - y0)*((x - m_data(0, i)) / (m_data(0, i + 1) - m_data(0, i)));
			}
			virtual DT limitHi() const override 
			{
				if (m_data.nc() == 0)throw FunctionNotInitialized();
				return m_data(0, m_data.nc() - 1);
			}
			virtual DT limitLo() const override 
			{
				if (m_data.nc() == 0)throw FunctionNotInitialized();
				return m_data(0, 0);
			}
			//reduce or increase the limit of the function
			virtual void setLimits(DT lo, DT hi) override
			{
				throw FunctionNotImplemented();
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				throw FunctionNotImplemented();
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				CT y;

				int imax = findIndex(xmax);
				y = f(xmax);
				ymin = y;
				ymax = y;

				int imin = findIndex(xmin);
				y = f(xmin);
				ymin = adore::mad::min<T,n,1>(ymin, y);
				ymax = adore::mad::max<T,n,1>(ymax, y);

				for (int i = imin + 1; i <= imax; i++)
				{
					y = subm(m_data, dlib::range(1, n), dlib::range(i, i));
					ymin = adore::mad::min<T,n,1>(ymin, y);
					ymax = adore::mad::max<T,n,1>(ymax, y);
				}
			}
		public: //other useful methods

			/**
			 * @brief shifts s to be in between limitLo and limitHi
			 * 
			 * @param s 
			 * @return double
			 * 
			 * @see limitLo(), limitHi() 
			 */
			virtual double limit_s_to_bounds(double s) const
			{
				    // s=adore::mad::bound(m_leftBorderDistance_fct.limitLo(),
					// m_leftBorderDistance_fct.limitLo()+s,
					// m_leftBorderDistance_fct.limitHi());
				return adore::mad::bound(limitLo(),limitLo()+s,limitHi());
			}

			/**
			 * retains the overall domain between lo and hi, but inverts direction in between
			 */
			virtual void invertDomain()
			{
				T old_lo = limitLo();
				T old_hi = limitHi();
				m_data = dlib::colm(m_data,m_data.nc()-dlib::range(1,m_data.nc()));
				set_rowm(m_data,0) = old_hi - rowm(m_data,0) + old_lo;
			}
			/**
			 * shifts the function in such a way that the domain starts at 0
			 */
			virtual void startDomainAtZero()
			{
				set_rowm(m_data,0) = rowm(m_data,0)-limitLo();
			}
			/**
			 * shifts the domain of the function by dx (move graph left or right)
			 */
			void shiftDomain(DT dx)
			{
				set_rowm(m_data,0) = rowm(m_data,0)+dx;
			}
			/**
			 * stretches domain to fit interval [x0,x1]
			 */
			virtual void stretchDomain(DT x0, DT x1)
			{
				shiftDomain(-limitLo());
				DT ratio = (x1-x0)/(limitHi()-limitLo());
				for(int i = 1; i < m_data.nc(); i++)
				{
					m_data(0,i) = m_data(0,i)*ratio;
				}
				shiftDomain(x0);
			}
			/**
			 * shifts the codomain of the function (move graph up or down)
			 */
			void shiftCodomain(CT dy)
			{
				auto rows = dlib::range(1,n);
				for(int i=0;i<m_data.nc();i++)
				{
					dlib::set_subm(m_data,rows,dlib::range(i,i)) = dlib::subm(m_data,rows,dlib::range(i,i)) + dy;
				}
			}
			/**
			 * shift a single dimension of the codomain
			 */
			void shiftCodomain(T dy,int i=0)
			{
				set_rowm(m_data,i+1) = rowm(m_data,i+1)+dy;
			}

			/**
			 * rotate codomain in 2D 
			 */
			void rotateXY(double angle, double x0=0.0, double y0=0.0)
			{
				for(int i=0 ; i<m_data.nc(); i++)
				{
					double x = m_data(1,i);
					double y = m_data(2,i);
					m_data(1,i) =  (x-x0)*std::cos(angle) - (y-y0)*std::sin(angle) + x0;
					m_data(2,i) =  (x-x0)*std::sin(angle) + (y-y0)*std::cos(angle) + y0;
				}
			}


			/**
			 * get a singular value from vector codomain type
			 */
			virtual T fi(DT x, int row) const override
			{
				if (m_data.nc() == 0)throw FunctionNotInitialized();
				int i = findIndex(x);
				return m_data(row + 1, i) + (m_data(row + 1, i + 1) - m_data(row + 1, i))*(T)(x - m_data(0, i)) / (T)(m_data(0, i + 1) - m_data(0, i));
			}
			/**
			 * numerical gradient: get a singular value from vector codomain type
			 */
			virtual T dfidx(DT x, int row)
			{
				if (m_data.nc() == 0)throw FunctionNotInitialized();
				int i = findIndex(x);
				T dy = m_data(row + 1, i + 1) - m_data(row + 1, i);
				T dx = m_data(0, i + 1) - m_data(0, i);
				return dy / dx;
			}
			/**
			 * sapmle a range of x values and evaluate gradient for one row of the vector only
			 */
			void sample_dfidx(DT* xvec, T* yvec, int count, int row)
			{
				if (m_data.nc() == 0)throw FunctionNotInitialized();
				for (int i = 0; i < count; i++)
				{
					yvec[i] = dfidx(xvec[i], row);
				}
			}
		private: 
			/**
			 * scalar versions of this function, for example evaluating to CT double not matrix
			 */
			class OneDimension :public ALFunction<DT, T>
			{
			private:
				LLinearPiecewiseFunctionM* m_parent;
				int m_row;
			public:
				OneDimension() {}
				OneDimension(LLinearPiecewiseFunctionM* parent, int row) :m_parent(parent), m_row(row) {}
				virtual T f(DT x)    const override { return m_parent->fi(x, m_row); }
				virtual DT limitHi() const override { return m_parent->limitHi(); }
				virtual DT limitLo() const override { return m_parent->limitLo(); }
				virtual void setLimits(DT lo, DT hi) { throw FunctionNotImplemented(); }
				virtual  ALFunction<DT, T>* clone()
				{
					return new LLinearPiecewiseFunctionS<T>(rowm(m_parent->m_data, 0), rowm(m_parent->m_data, m_row + 1));
				}
				virtual ALFunction<DT, T>* create_derivative()override
				{
					throw FunctionNotImplemented();
				}
				virtual void bound(const DT& xmin, const DT& xmax, T& ymin, T& ymax) override
				{
					T y;

					int imax = m_parent->findIndex(xmax);
					y = m_parent->fi(xmax, m_row);
					ymin = y;
					ymax = y;

					int imin = m_parent->findIndex(xmin);
					y = m_parent->fi(xmin, m_row);
					ymin = adore::mad::min(ymin, y);
					ymax = adore::mad::max(ymax, y);

					for (int i = imin + 1; i <= imax; i++)
					{
						y = m_parent->m_data(m_row + 1, i);
						ymin = adore::mad::min(ymin, y);
						ymax = adore::mad::max(ymax, y);
					}
				}
			};
			OneDimension single_dimensions[n];
		public: 
			/**
			 * get access to a scalar version of this function
			 */
			virtual ALFunction<DT, T>* dimension(int i) override
			{
				return &single_dimensions[i];
			}
		public:
			/**
			 * constructor without initialization
			 */
			LLinearPiecewiseFunctionM()
			{
				m_searchIndex = 0;
				for (int i = 0; i < n; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
			}
			LLinearPiecewiseFunctionM& operator=(const LLinearPiecewiseFunctionM& other)
			{
				if(&other==this)return *this;
				this->m_data = other.m_data;
				this->m_searchIndex = other.m_searchIndex;
				for (int i = 0; i < n; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
				return *this;
			}
			/**
			 * copy constructor
			 */
			LLinearPiecewiseFunctionM(const LLinearPiecewiseFunctionM& other)
			{
				this->m_data = other.m_data;
				m_searchIndex = other.m_searchIndex;
				for (int i = 0; i < n; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
			}
			/**
			 * constructor: initialize with data matrix containing domain and codomain
			 */
			LLinearPiecewiseFunctionM(const adoreMatrix<T, n + 1, 0>& data)
			{
				this->m_data = data;
				m_searchIndex = 0;
				for (int i = 0; i < n; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
			}
			/**
			 * constructor: initialize with two data matrices containing domain and codomain separately
			 */
			LLinearPiecewiseFunctionM(const adoreMatrix<T, 1, 0>& xdata,const adoreMatrix<T, n , 0>& ydata)
			{
				dlib::set_rowm(this->m_data,0) = xdata;
				dlib::set_rowm(this->m_data,dlib::range(1l,n+1l)) = ydata;
				m_searchIndex = 0;
				for (int i = 0; i < n; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
			}
			/**
			 * constructor: dynamically sized matrix, initialized with a single value
			 */
			LLinearPiecewiseFunctionM(const int nc,T value)
			{
				this->m_data.set_size(n+1,nc);
				this->m_data = dlib::ones_matrix<T>(n+1,nc)*value;
				m_searchIndex = 0;
				for (int i = 0; i < n; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
			}
			virtual ~LLinearPiecewiseFunctionM() {}
			virtual ALFunction<DT, CT>* clone() override
			{
				return new LLinearPiecewiseFunctionM<T, n>(this->m_data);
			}
			//sorry, cannot be virtualized due to c++ limitations, first parameter cannot be removed due to how variadric arguments work, no templated friend method of templated class.
/*			template<int Nnew>
			LLinearPiecewiseFunctionM<T, Nnew>* clone(int first, va_list args)
			{
				adoreMatrix<T, Nnew + 1, 0> clone_data;
				clone_data = dlib::zeros_matrix<T>(Nnew + 1, m_data.nc());
				set_rowm(clone_data, 0) = rowm(m_data, 0);//copy x
				set_rowm(clone_data, 1) = rowm(m_data, first);//copy first index

				for (int i = 0; i < Nnew - 1; i++)
				{
					int dim = va_arg(args, int );
					set_rowm(clone_data, i + 2) = rowm(m_data, dim + 1);//copy yi
				}
				va_end(args);
				return new LLinearPiecewiseFunctionM<T, Nnew>(clone_data);
			}
			template<int Nnew>
			LLinearPiecewiseFunctionM<T, Nnew>* clone(int first, ...)
			{
				va_list args;
				va_start(args, first);
				return clone<Nnew>(first, args);
			}
*/
			/**
			 * set the domain and codomain from data matrix
			 */
			void setData(const adoreMatrix<T, n + 1, 0>& data)
			{
				this->m_data = data;
			}

		public://operators
			/**
			 *  apply operation to function sub-dimensions: multiply with matrix of lower dimension in range rowi to rowj, with A.nc==A.nr==rowj-rowi+1
			 */
			virtual void multiply(adoreMatrix<T, 0, 0> A, int rowi, int rowj) override
			{
				assert(A.nc() == A.nr());
				assert(A.nr() == rowj - rowi + 1);
				assert(rowi >= 0);
				assert(rowj < n);

				set_rowm(m_data, dlib::range(rowi + 1, rowj + 1)) = A * rowm(m_data, dlib::range(rowi + 1, rowj + 1));
			}
			/**
			 *  apply operation to function sub-dimensions: add vector b to dimensions rowi to rowj
			 */
			virtual void add(adoreMatrix<T, 0, 1> b, int rowi, int rowj) override
			{
				assert(b.nr() == rowj - rowi + 1);
				assert(rowi >= 0);
				assert(rowj < n);
				for (int i = 0; i < m_data.nc(); i++)
				{
					set_subm(m_data, dlib::range(rowi + 1, rowj + 1), dlib::range(i, i)) = subm(m_data, dlib::range(rowi + 1, rowj + 1), dlib::range(i, i)) + b;
				}
			}

		public://intersection test and other useful operations
			/**
			 *	test whether this and other intersect in dimension d1 and d2 - a vector of pairs of x-values (domain) is returned,
			 *	with one entry for every intersection and the first entry in a pair for this and the second entry for other
			 */
			std::vector<std::pair<T, T> >* getIntersections2d(LLinearPiecewiseFunctionM<T, n>* other, int dim1, int dim2)
			{
				std::vector<std::pair<T, T> >* result = new std::vector<std::pair<T, T>>();
				T a0, b0, c0, d0, a1, b1, c1, d1, e0, e1, f0, f1;
				T xi0, xi1, xj0, xj1, xi, xj,alpha,beta;
				for (int i = 0; i < this->m_data.nc() - 1; i++)
				{
					xi0 = this->m_data(0, i);
					xi1 = this->m_data(0, i+1);
					a0 = this->m_data(dim1 + 1, i);
					a1 = this->m_data(dim2 + 1, i);
					b0 = this->m_data(dim1 + 1, i + 1);
					b1 = this->m_data(dim2 + 1, i + 1);
					e0 = (b0 - a0) ;
					e1 = (b1 - a1) ;
					for (int j = 0; j < other->m_data.nc() - 1; j++)
					{
						xj0 = other->m_data(0, j);
						xj1 = other->m_data(0, j+1);
						c0 = other->m_data(dim1 + 1, j);
						c1 = other->m_data(dim2 + 1, j);
						d0 = other->m_data(dim1 + 1, j + 1);
						d1 = other->m_data(dim2 + 1, j + 1);
						f0 = (d0 - c0);
						f1 = (d1 - c1);
						if (e1 * f0 - e0 * f1 == 0) continue;//parallel lines
						alpha	= (c0*f1-c1*f0-a0*f1+a1*f0) / (e0*f1-e1*f0);
						beta	= (a0*e1-a1*e0-c0*e1+c1*e0) / (e1*f0-e0*f1);
						if(		0.0<=alpha	&& alpha<=1.0 
							&&	0.0<=beta	&& beta<=1.0	)
						{
							xi = xi0+alpha*(xi1-xi0);
							xj = xj0+beta*(xj1-xj0);
							result->push_back(std::pair<T, T>(xi,xj));//intersection in interval
						}
					}
				}
				return result;
			}
			std::vector<std::pair<T, T> >* getIntersections2d(LLinearPiecewiseFunctionM<T, n>* other)
			{
				return getIntersections2d(other, 0, 1);
			}
			/**
			 *	computes the first intersection between two line sequences, with first seen from perspective of this
			 *	returns true if an intersection was found and then writes domain values for intersection into result, with first for this and second for other.
			 */
			bool getFirstIntersection2d(LLinearPiecewiseFunctionM<T, n>* other, int dim1, int dim2, std::pair<T,T>& result)
			{
				T a0, b0, c0, d0, a1, b1, c1, d1, e0, e1, f0, f1;
				T xi0, xi1, xj0, xj1, alpha,beta;
				for (int i = 0; i < this->m_data.nc() - 1; i++)
				{
					xi0 = this->m_data(0, i);
					xi1 = this->m_data(0, i+1);
					a0 = this->m_data(dim1 + 1, i);
					a1 = this->m_data(dim2 + 1, i);
					b0 = this->m_data(dim1 + 1, i + 1);
					b1 = this->m_data(dim2 + 1, i + 1);
					e0 = (b0 - a0);
					e1 = (b1 - a1);
					for (int j = 0; j < other->m_data.nc() - 1; j++)
					{
						xj0 = other->m_data(0, j);
						xj1 = other->m_data(0, j+1);
						c0 = other->m_data(dim1 + 1, j);
						c1 = other->m_data(dim2 + 1, j);
						d0 = other->m_data(dim1 + 1, j + 1);
						d1 = other->m_data(dim2 + 1, j + 1);
						f0 = (d0 - c0);
						f1 = (d1 - c1);
						if (std::abs(e1 * f0 - e0 * f1 )< 1.0e-3) continue;//parallel lines
						alpha	= (c0*f1-c1*f0-a0*f1+a1*f0) / (e0*f1-e1*f0);
						beta	= (a0*e1-a1*e0-c0*e1+c1*e0) / (e1*f0-e0*f1);
						if(		0.0<=alpha	&& alpha<=1.0 
							&&	0.0<=beta	&& beta<=1.0	)
						{
							double dx = a0 + e0*alpha - (c0 + f0*beta);
							double dy = a1 + e1*alpha - (c1 + f1*beta);
							if(dx*dx+dy*dy>1e-4){std::cout << "Llinearpiecewise l 635"<<std::endl;continue;}
							result.first = xi0+alpha*(xi1-xi0);
							result.second = xj0+beta*(xj1-xj0);
							return true;//intersection in interval
						}
					}
				}
				return false;
			}
			bool getFirstIntersection2d(LLinearPiecewiseFunctionM<T, n>* other, std::pair<T,T>& result)
			{
				return getFirstIntersection2d(other, 0, 1,result);
			}

			/**
			* getFirstIntersection1d - zero crossing test
			*	returns true, if a "zero" crossing happens
			*	tests for crossing y_cross value in dimension "dimension"
			*	the x coordinate is written in x, if a crossing occurs
			*/
			bool getFirstIntersection1d(T y_cross,int dimension,T& x)
			{
				double dir = m_data(dimension,0)>y_cross?1.0:-1.0;
				for(int i=1;i<m_data.nc();i++)
				{
					if( m_data(dimension,i)*dir<y_cross*dir )
					{
						x = (y_cross-m_data(dimension,i-1))/(m_data(dimension,i)-m_data(dimension,i-1))*(m_data(0,i)-m_data(0,i-1))+m_data(0,i-1);
						return true;
					}
				}
				return false;
			}
			bool getFirstIntersection1d(T y_cross,T& x)
			{
				return getFirstIntersection1d(y_cross,1,x);
			}
			/**
			 * one dimensional intersection test
			 * @param dim dimension to be tested
			 * @param result first intersection: domain values of intersection point
			 * @return true if other and this intersect in dimension dim
			 */
			bool getFirstIntersection1d(LLinearPiecewiseFunctionM<T, n>* other,int dim,std::pair<T,T>& result)
			{
				double xstart = (std::max)(this->limitLo(),other->limitLo());
				double xend = (std::min)(this->limitHi(),other->limitHi());
				if(xstart<xend)
				{
					int i = this->findIndex(xstart);
					int j = other->findIndex(xstart);
					bool direction = this->fi(xstart,dim)<other->fi(xstart,dim);
					while(i<this->m_data.nc() && j<other->m_data.nc())
					{
						double x = adore::mad::bound((std::min)(this->limitLo(),other->limitLo()),(std::max)(this->m_data(0,i),other->m_data(0,j)), (std::min)(this->limitHi(),other->limitHi()));
						bool new_direction = this->fi(x,dim)<other->fi(x,dim);
						double x_this = this->getData()(0,i);
						double x_other = other->getData()(0,j);
						if( new_direction!=direction )
						{
							if(x_this<x_other)
							{
								i--;
							}
							else
							{
								j--;
							}
							i = (std::min<unsigned int>)(this->m_data.nc()-1,i);
							j = (std::min<unsigned int>)(other->m_data.nc()-1,j);
							double xthis;
							double xother;
							adore::mad::intersectLines2(this->m_data(0,i),this->m_data(dim+1,i),this->m_data(0,i+1),this->m_data(dim+1,i+1),
													  other->m_data(0,j),other->m_data(dim+1,j),other->m_data(0,j+1),other->m_data(dim+1,j+1),
													  xthis,xother,0.0);
							result.first = this->m_data(0,i) + (this->m_data(0,i+1)-this->m_data(0,i))*xthis;
							result.second = other->m_data(0,j) + (other->m_data(0,j+1)-other->m_data(0,j))*xother;
							return true;
						}
						//double x_this = this->getData()(0,i);
						//double x_other = other->getData()(0,j);
						if(x_this<x_other)
						{
							i++;
						}
						else
						{
							j++;
						}
					}
					return false;
				}
				else
				{
					return false;
				}
			}

			/**
			 *	getNextIntersectionWithLine2d - returns the next x-position after x0, at which the line with start (px,py) and direction (vx,vy) intersects this in dimensions d1,d2.
			 *	returns true if an intersection was found after x0
			 *  sets x_result to the parametrization of this at which intersection occurs, e.g. ip = f_d1d2(x_result);
			 *  sets distance to the parameter required to achieve the intersection point via the given line, e.g. ip=(px,py) + (vx,vy)*distance;
			 *  extend_fringes - if extend_fringes is true, the function is extrapolated in order to provide intersection points beyond its domain
			 */
			bool getNextIntersectionWithLine2d(T x0, T px, T py, T vx, T vy, int d1, int d2, T& x_result, T& distance,bool extend_fringes=false,bool inside_input_line=false)
			{
				int i0 = findIndex(x0);
				double xa, xb;
				bool xa_inside;
				bool xb_inside;
				for (int i = i0; i < m_data.nc() - 1; i++)
				{
					adore::mad::intersectLines(px, py, px + vx, py + vy,
						m_data(d1 + 1, i), m_data(d2 + 1, i), m_data(d1 + 1, i + 1), m_data(d2 + 1, i + 1),
						xa, xb, xa_inside, xb_inside);
					if (xb_inside && (!inside_input_line || xa_inside))
					{
						x_result = m_data(0, i) + (m_data(0, i + 1) - m_data(0, i)) * xb;
						distance = xa;
						return true;
					}
					else
					{
						if(extend_fringes)
						{
							//intersection before start of domain?
							if(i==0 && xb<=0)
							{
								x_result = limitLo();
								distance = xa;
								return true;
							}
							if( i==m_data.nc()-1 && xb>1 )
							{
								x_result = limitHi();
								distance = xa;
								return true;
							}
						}
					}
				}
				return false;
			}
			bool getNextIntersectionWithLine2d(T x0, T px, T py, T vx, T vy, T& x_result, T& distance, bool extend_fringes=false)
			{
				return getNextIntersectionWithLine2d(x0, px, py, vx, vy, 0, 1, x_result, distance,extend_fringes);//for dimension 0 and 1
			}
			/*
			 *	takes an initial search location x0, 
			 *  a point px,py and a unit vector vx,vy
			 *  returns true if an intersection can be found with |distance|<max_distance
			 *	returns also the intersection parameter x_result and the true distance between px,py and the intersection point, with a positive distance indicating in direction of vector and negative distance the opposite
			 */
			bool getNextIntersectionWithVector2d(T x0, T px, T py, T vx, T vy, T& x_result, T& distance, T max_distance,bool extend_fringes=false)
			{
				T s;//parameter for line
				T px1,py1,vx1,vy1;//shifted+stretched values
				px1 = px-max_distance*vx;
				py1 = py-max_distance*vy;
				vx1 = ((T)2)*max_distance*vx;
				vy1 = ((T)2)*max_distance*vy;
				bool rv = getNextIntersectionWithLine2d(x0,px1,py1,vx1,vy1, 0, 1, x_result,s,extend_fringes,true);
				if(rv)
				{
					distance = ((T)2)*max_distance*s-max_distance;
				}
				else
				{
					distance = max_distance;
				}
				return rv;
			}


			/**
			 * countIntersectionsWithRay2d - sends a ray starting at px,py in direction of x axis and counts intersections with the lines of this object.
			 * dimensions d1 and d2 are used.
			 * furthermore, an extension point (ex,ey) is given, which allows to test another line from last point to (ex,ey)
			 * The method can be used to test for a point (px,py) whether it is inside the polygon defined by this function object: Unequal number of intersections <=> point is inside
			 * specify (ex,ey) as first point of this function object to close the loop of the line sequence.
			 * specify (ex,ey) as first point fo next function object to test line sequence consisting of multiple function objects
			 * https://en.wikipedia.org/wiki/Point_in_polygon
			 */
			int countIntersectionsWithRay2d(T px, T py, int d1, int d2, T ex, T ey,bool inverted = false)
			{
				double count = 0;
				double rx,ry;//lower point
				double sx,sy;//higher point
				double x0;//x-value at intersection of x-axis

				for (int i = 0; i < m_data.nc() - 1; i++)
				{
					//order points i and i+1 by y-value
					if(m_data(d2,i)<m_data(d2,i+1))
					{
						rx = m_data(d1,i)-px;	ry=m_data(d2,i)-py;
						sx = m_data(d1,i+1)-px;	sy=m_data(d2,i+1)-py;
					}
					else
					{
						if(m_data(d2,i+1)<m_data(d2,i))
						{
						rx = m_data(d1,i+1)-px;	ry=m_data(d2,i+1)-py;
						sx = m_data(d1,i)-px;	sy=m_data(d2,i)-py;
						}
						else
						{
							continue;//no delta y
						}
					}
					if(ry<(T)0 && (T)0<sy)//is there a zero transition?
					{
						//find intersection with x-axis
						x0 = rx-ry*(sx-rx)/(sy-ry);
						if(x0>(T)0)
						{
							count++;
						}
					}
				}

				//extension point
				int i=(inverted)?0:m_data.nc()-1;
				if(m_data(d2,i)<ey)
				{
					rx = m_data(d1,i)-px;	ry=m_data(d2,i)-py;
					sx = ex-px;				sy=ey-py;
				}
				else
				{
					if(ey<m_data(d2,i))
					{
					rx = ex-px;				ry=ey-py;
					sx = m_data(d1,i)-px;	sy=m_data(d2,i)-py;
					}
					else
					{
						return count;//no delta y
					}
				}
				if(ry<(T)0 && (T)0<sy)//is there a zero transition?
				{
					//find intersection with x-axis
					x0 = rx-ry*(sx-rx)/(sy-ry);
					if(x0>(T)0)
					{
						count++;
					}
				}


				return count;
			}

			/**
			 *	getPositionOfPoint - returns best-fit d_tangential and d_normal: if d_tangential>1 or d_tangential<0, the point is not projected onto function interval
			 *	the according function parameter for given point is returned
			 */
			double getPositionOfPoint(T px, T py, int d1, int d2, T& d_tangential_min, T& d_normal_min)
			{
				T DIST_GUARD = 1e99;
				T d_tangential_i;
				T d_normal_i;
				d_normal_min = DIST_GUARD;
				d_tangential_min = DIST_GUARD;
				T xmin = limitLo();

				for (int i = 0; i < m_data.nc() - 1; i++)
				{
//					comparePointWithLine(m_data(d1, 0), m_data(d2, 0), m_data(d1, 1), m_data(d2, 1), px, py, d_tangential_i, d_normal_i);
					comparePointWithLine(m_data(d1, i), m_data(d2, i), m_data(d1, i+1), m_data(d2, i+1), px, py, d_tangential_i, d_normal_i);
					if ((T)0 <= d_tangential_i && d_tangential_i <= (T)1)//in interval
					{
						if ((T)0 <= d_tangential_min && d_tangential_min <= (T)1)//previously in interval
						{
							if ((std::abs)(d_normal_i) < (std::abs)(d_normal_min))//smaller normal distance
							{
								d_normal_min = d_normal_i;
								d_tangential_min = d_tangential_i;
								xmin = m_data(0,i) + (m_data(0,i+1)-m_data(0,i)) * d_tangential_min;
								xmin = (std::max)(limitLo(),(std::min)(xmin,limitHi()));
							}
						}
						else//previously not in interval
						{
							d_normal_min = d_normal_i;
							d_tangential_min = d_tangential_i;
							xmin = m_data(0,i) + (m_data(0,i+1)-m_data(0,i)) * d_tangential_min;
							xmin = (std::max)(limitLo(),(std::min)(xmin,limitHi()));
						}
					}
					else//not in interval
					{
						if (d_tangential_min < (T)0 || (T)1 < d_tangential_min)//previously not in interval
						{
							if ((std::abs)(d_tangential_i - (T)0.5) < (std::abs)(d_tangential_min - (T)0.5))//smaller distance to center of interval
							{
								d_normal_min = d_normal_i;
								d_tangential_min = d_tangential_i;
								if(d_tangential_min<(T)0)
								{
									xmin = limitLo();
								}
								else
								{
									xmin = limitHi();
								}
							}
						}
					}
				}
				return xmin;
			}

			/**
			 *  returns the parameter of the function value at dimensions d1,d2 with closest euclidean distance to px,py
			 */
			double getClosestParameter(T px, T py, int d1, int d2,T& n_min) const
			{
				T rel,x_min=limitLo();
				T d, d_min = 1e99,normal;
				for(int i=0;i<m_data.nc()-1;i++)
				{
					d = getDistancePointToLine(m_data(d1, i), m_data(d2, i), m_data(d1, i+1), m_data(d2, i+1), px, py, rel,normal);
					if( d<d_min )
					{
						d_min = d;
						n_min = normal;
						x_min = m_data(0,i) + (m_data(0,i+1)-m_data(0,i)) * rel;
					}
				}
				x_min = (std::max)(limitLo(),(std::min)(x_min,limitHi()));
				return x_min;
			}
			/**
			 *  returns the parameter of the function value at dimensions d1,d2 with closest euclidean distance to px,py, local search starting at x0
			 */
			double getClosestParameter_local(T px, T py, int d1, int d2,T x0,T* n_min=nullptr) const
			{
				int i0 = std::max(0,(int)findIndex(x0));
				T rel,x_min=limitLo();
				T d, d_min = 1e99,normal;
				d_min = getDistancePointToLine(m_data(d1, i0), m_data(d2, i0), m_data(d1, i0+1), m_data(d2, i0+1), px, py, rel,normal);
				x_min = m_data(0,i0) + (m_data(0,i0+1)-m_data(0,i0)) * rel;

				for(int i=i0+1;i<m_data.nc()-1;i++)
				{
					if(		std::abs(m_data(d1,i+1)-m_data(d1,i))<1e-10
						&&	std::abs(m_data(d2,i+1)-m_data(d2,i))<1e-10	)continue;
					d = getDistancePointToLine(m_data(d1, i), m_data(d2, i), m_data(d1, i+1), m_data(d2, i+1), px, py, rel,normal);
					if( d<d_min )
					{
						d_min = d;
						x_min = m_data(0,i) + (m_data(0,i+1)-m_data(0,i)) * rel;
						if(n_min!=nullptr)*n_min=normal;
					}
					else
					{
						break;
					}
				}
				x_min = (std::max)(limitLo(),(std::min)(x_min,limitHi()));
				return x_min;
			}

			double getClosestParameter(T px, T py, int d1, int d2) const
			{
				T d_min;
				return getClosestParameter( px,  py,  d1,  d2, d_min);
			}

			
			/**
			 * isPointEnclosed - Returns true if a point x is enclosed by the two line segments this, other and the lines connecting this' and other's endpoints.
			 * This and other have to form a counter clockwise loop if their endpoints are connected, so that the interior of the loop is well defined.
			 * If this and other do not form a ccw loop, use invert_this or invert_other to temporarily change directions, to achieve ccw loop.
			 * https://en.wikipedia.org/wiki/Point_in_polygon
			 */
			bool isPointEnclosed(LLinearPiecewiseFunctionM<T, n>* other, T px, T py, int d1, int d2, bool invert_this = false, bool invert_other = true)
			{

				//extension point for this and other (making the connection between the two line sequences)
				double ex0,ey0;
				double ex1,ey1;
				if(invert_other)
				{
					ex0 = other->m_data(d1,other->m_data.nc()-1);
					ey0 = other->m_data(d2,other->m_data.nc()-1);
				}
				else
				{
					ex0 = other->m_data(d1,0);
					ey0 = other->m_data(d2,0);
				}
				if(invert_this)
				{
					ex1 = this->m_data(d1,this->m_data.nc()-1);
					ey1 = this->m_data(d2,this->m_data.nc()-1);
				}
				else
				{
					ex1 = this->m_data(d1,0);
					ey1 = this->m_data(d2,0);
				}

				//sum the number of intersections with an arbitrary ray starting at px,py
				int count = 0;
				count += this->countIntersectionsWithRay2d(px,py,d1,d2,ex0,ey0,invert_this);
				count += other->countIntersectionsWithRay2d(px,py,d1,d2,ex1,ey1,invert_other);

				return (count%2)!=0;
			}

			/**
			 *	returns the x-value if going forward N points
			 */
			T getXAfterNPoints(T xstart, int N)
			{
				xstart = (std::max)(xstart,limitLo());
				int i0 = this->findIndex(xstart);
				int i1 = (std::min)(i0 + N - 1, (int) m_data.nc()-1);
				return m_data(0, i1);
			}

			/**
			 *	writes j-i points into an array m, using the natural stepsize
			 */
			void writePointsToArray(int i,int j, T* m)
			{
				for(int k=i;k<=j;k++)
				{
					for(int d = 0; d<n; d++)
					{
						m[(k-i)*n+d] = m_data(d+1,k);
					}
				}
			}
			/**
			 *	writes j-i points into an array m, for dimension d, using the natural stepsize
			 */
			void writePointsToArray(int i,int j, int d, T* m)
			{
				for(int k=i;k<=j;k++)
				{
					m[k-i] = m_data(d+1,k);
				}
			}

			/**
			 *	export points between x0 and x1 values to a matrix 
			 */
			int export_points(adoreMatrix<double,0,0>& target, double x0, double x1,double precision)
			{
				x0 = (std::max)(x0,limitLo());
				x1 = (std::min)(x1,limitHi());
				int i = findIndex(x0);
				int j = findIndex(x1);
				//if the distance between x0 and m_data(:,i) is low, use m_data(:,i) as first point
				if((std::abs)(m_data(0,i)-x0)<precision)
				{
					set_subm(target,dlib::range(0,n),dlib::range(0,0)) = subm(m_data,dlib::range(0,n),dlib::range(i,i));
					i ++;
				}
				else
				{
					//if the distance between x0 and m_data(:,i+1) is low, use m_data(:,i+1) as first point
					int l =(std::min)(i+1,(int)m_data.nc());
					if((std::abs)(m_data(0,l)-x0)<precision)
					{
						set_subm(target,dlib::range(0,n),dlib::range(0,0)) = subm(m_data,dlib::range(0,n),dlib::range(l,l));
						i+=2;
					}
					//otherwise, evaluate f(x0) and use this intermediate point as first point in target
					else
					{
						target(0,0) = x0;
						set_subm(target,dlib::range(1,n),dlib::range(0,0)) = f(x0);
						i++;
					}
				}
				//set intermediate points, if i<=j
				if(i<=j)
				{
					set_subm(target,dlib::range(0,n),dlib::range(1,j-i+1)) = subm(m_data,dlib::range(0,n),dlib::range(i,j));
				}
				if(x1 - m_data(0,j)>precision || i>j)
				{
					target(0,j-i+2) = x1;
					set_subm(target,dlib::range(1,n),dlib::range(j-i+2,j-i+2)) = f(x1);
					return j-i+3;
				}
				else
				{
					return j-i+2;
				}
			}

		};

		/**
		 * LLinearPiecewiseFunctionA - an array based linear piecewise function
		 * the array T* m_data is assumed to contain data for an N by k matrix, where an entry is located at m_data[col*N+row]
		 * the x data is assumed to be spaced equidistantly with m_dx between m_x0 and m_x1, leading to k points.
		 */
		template<typename T, int N, int k>
		class LLinearPiecewiseFunctionA :public AScalarToN<T, N>
		{
		public:
			typedef typename AScalarToN<T,N>::DT DT;
			typedef typename AScalarToN<T,N>::SUBFUN SUBFUN;
		private:
			T m_data[N*k];
			DT m_x0;
			DT m_x1;
			DT m_dx;
			inline void idx(DT x, int dim, int & i, int & j) const
			{
				int c = (std::max)((std::min)(0, (int)(std::floor)((x - m_x1) / m_dx)), k - 1);
				i = c*N + dim;
				j = (c + 1)*N + dim;
			}
			inline DT xval(int i)
			{
				return m_x0 + m_dx * i;
			}
		public:
			typedef adoreMatrix<T, N, 1> CT;
			virtual CT f(DT x) const override
			{
				CT m;
				for (int i = 0; i < N; i++)m(i, 1) = fi(x, i);
				return m;
			}
			virtual T fi(DT x, int dim) const override
			{
				int i, j;
				idx(x, dim, i, j);
				return m_data[i] + (x - xval(i)) / m_dx * (m_data[j] - m_data[i]);
			}
			virtual DT limitHi() const override
			{
				return m_x1;
			}
			virtual DT limitLo()const override
			{
				return m_x0;
			}
			//virtual void invertDirection()override
			//{
			//	throw FunctionNotImplemented();
			//}
			virtual void setLimits(DT lo, DT hi)override
			{
				throw FunctionNotImplemented();
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				throw FunctionNotImplemented();
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax) override
			{
				CT y;

				int imax, jmax;
				idx(xmax, 0, imax, jmax);
				y = f(xmax);
				ymin = y;
				ymax = y;

				int imin, jmin;
				idx(xmin, 0, imin, jmin);
				y = f(xmin);
				ymin = adore::mad::min(ymin, y);
				ymax = adore::mad::max(ymax, y);

				for (int i = jmin; i <= imax; i += N)
				{
					for (int j = 0; j < N; j++)
					{
						T yj = m_data[i*N + j];
						ymin(j) = (std::min)(ymin(j), yj);
						ymax(j) = (std::max)(ymax(j), yj);
					}
				}
			}
		private: //scalar versions of this function, for example evaluating to CT double not matrix
			class OneDimension :public ALFunction<DT, T>
			{
			private:
				LLinearPiecewiseFunctionA* m_parent;
				int m_row;
				typedef ALFunction<DT,T> SUBFUN;
			public:
				OneDimension() {}
				OneDimension(LLinearPiecewiseFunctionA* parent, int row) :m_parent(parent), m_row(row) {}
				virtual T f(DT x) override { return m_parent->fi(x, m_row); }
				virtual DT limitHi() const override { return m_parent->limitHi(); }
				virtual DT limitLo() const override { return m_parent->limitLo(); }
				virtual void setLimits(DT lo, DT hi)override { throw FunctionNotImplemented(); }
				virtual SUBFUN* clone()override { throw FunctionNotImplemented(); }
				virtual SUBFUN* create_derivative()override { throw FunctionNotImplemented(); }
				virtual void bound(const DT& xmin, const DT& xmax, T& ymin, T& ymax) override
				{
					T y;

					int imax, jmax;
					m_parent->idx(xmax, m_row, imax, jmax);
					y = m_parent->fi(xmax, m_row);
					ymin = y;
					ymax = y;

					int imin, jmin;
					m_parent->idx(xmin, m_row, imin, jmin);
					y = m_parent->fi(xmin, m_row);
					ymin = (std::min)(ymin, y);
					ymax = (std::max)(ymax, y);

					for (int i = jmin; i <= imax; i += N)
					{
						y = m_parent->m_data[i*N + m_row];
						ymin = (std::min)(ymin, y);
						ymax = (std::max)(ymax, y);
					}
				}
			};
			OneDimension single_dimensions[N];
		public: //get access to a scalar version of this function
			virtual SUBFUN* dimension(int i) override
			{
				return &single_dimensions[i];
			}
			LLinearPiecewiseFunctionA(T* data, T x0, T x1)
			{
				m_x0 = x0;
				m_x1 = x1;
				m_dx = (x1 - x0) / (T)k;
				memcpy(m_data, data, sizeof(T)*N*k);
				for (int i = 0; i < N; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
			}
			virtual ~LLinearPiecewiseFunctionA() {}
			virtual ALFunction<DT, CT>* clone() override
			{
				return new LLinearPiecewiseFunctionA<T, N, k>(this->m_data, m_x0, m_x1);
			}

			/**
			*  apply operation to function: multiply with matrix
			*/
			virtual void multiply(adoreMatrix<T, 0, 0> A, int row_start, int row_end) override
			{
				assert(A.nc() == A.nr());
				assert(A.nr() == row_end - row_start + 1);
				assert(row_start >= 0);
				assert(row_end < N);

				T buf[N];
				for (int col = 0; col < k; col++)
				{
					for (int i = row_start; i <= row_end; i++)
					{
						buf[i] = (T)0;
						for (int j = row_start; j <= row_end; j++)buf[i] += A(i - row_start, j - row_start)*m_data[col*N + j];
					}
					for (int i = row_start; i <= row_end; i++)
					{
						m_data[col*N + i] = buf[i];
					}
				}
			}
			virtual void add(adoreMatrix<T, 0, 1> b, int row_start, int row_end) override
			{
				assert(b.nr() == row_end - row_start + 1);
				assert(row_start >= 0);
				assert(row_end < N);
				for (int col = 0; col < k; col++)
				{
					for (int i = row_start; i <= row_end; i++)
					{
						m_data[col*N + i] += b(i - row_start, 0);
					}
				}
			}
		};

		template<typename T, int dout,int din>
		LLinearPiecewiseFunctionM<T,dout> getSubFunction(LLinearPiecewiseFunctionM<T,din>& fin,int dimensions[dout])
		{
			adoreMatrix<T> m_out;
			m_out.set_size(dout+1,fin.getData().nc());
			dlib::set_rowm(m_out,0) = dlib::rowm(fin.getData(),0);
			for(int i=0;i<dout;i++)
			{
				dlib::set_rowm(m_out,i+1) = dlib::rowm(fin.getData(),dimensions[i]+1);
			}
			return LLinearPiecewiseFunctionM<T,dout>(m_out);
		}

		/**
			*	defineDistanceMap - defines dfun's i-th row to be d(t), with target(s)=base(t)+normal(t)*d(t)*scale. 
			*  d is computed for each sampling point of base. If dfun.nc() is smaller than base nc(), this data is resized and therefore all previous values are lost. 
			*  this sampling is set to equal the sampling of base. If target or n have smaller domains than base, values outside their domain are assigned with either guard or guardfun.
			*  if guardfun==0, guard constant guard value is used outside domain
			*  otherwise, guardfun is sampled 
			*/
		template<typename T,int n_dfun,int n_base,int n_normal,int n_target>
		void defineDistanceMap2d(LLinearPiecewiseFunctionM<T,n_dfun>* dfun,int id,LLinearPiecewiseFunctionM<T,n_base>* base, LLinearPiecewiseFunctionM<T,n_normal>* normal, LLinearPiecewiseFunctionM<T,n_target>* target,T guard,LLinearPiecewiseFunctionM<T,1>* guardfun=0)
		{
			static const int it = 0;//maybe move this to template
			static const int ix = 1;//maybe move this to template
			static const int iy = 2;//maybe move this to template
			if(base->getData().nc()!=dfun->getData().nc())
			{
				dfun->getData().set_size(dfun->getData().nr(),base->getData().nc());
			}
			dlib::set_rowm(dfun->getData(),it) = dlib::rowm(base->getData(),it);//set dfun's sampling equal to base's
			T s = target->limitLo();
			for(int j=0;j<dfun->getData().nc();j++)
			{
				double dj;
				target->getNextIntersectionWithVector2d(s, base->getData()(ix,j),
														   base->getData()(iy,j),
														   normal->getData()(ix,j),
														   normal->getData()(iy,j),
														s,dj,guard);
				if( guardfun!=0 && dj==guard )
				{
					dj = guardfun->f(adore::mad::bound(guardfun->limitLo(),s,guardfun->limitHi()));
				}
				dfun->getData()(id,j) = dj;
			}
		}
	}
}