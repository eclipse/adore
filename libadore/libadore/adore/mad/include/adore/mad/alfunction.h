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
#include <adore/mad/adoremath.h>
#include <adore/mad/intervalarithmetic.h>
#include <stdexcept>
#include <string>

namespace adore
{
	namespace mad
	{

		/**
		 * Exception occurs if a function with a bounded codomain is manipulated outside its domain
		 */
		class FunctionOutOfBoundsException : public std::runtime_error
		{
		public:
			FunctionOutOfBoundsException() :runtime_error("A bounded function of type ALFunction was accessed out of its bounds.") {}
		};
		/**
		 * Runtime error occuring when operations on functions are not implemented
		 */
		class FunctionNotImplemented : public std::runtime_error
		{
		public:
			FunctionNotImplemented() :runtime_error("An function has been called on an ALFunction object, which is not implemented.") {}
		};
		/**
		 * Error occurs when function is manipulated without initialization
		 */
		class FunctionNotInitialized : public std::runtime_error
		{
		public:
			FunctionNotInitialized() :runtime_error("An operation has been called on an ALFunction object, although the object is not properly initialized.") {}
		};
		/**
		 * Error occurs if a piecewise function cannot index the corresponding piece.
		 */
		class FunctionIndexingError : public std::runtime_error
		{
		public:
			FunctionIndexingError() :runtime_error("A function was unable to index a value x.") {}
		};
		/**
		 * Error occurs when bounds in a given interval are queried, but the function is onbounded on the interval.
		 */
		class FunctionUnboundedException :public std::runtime_error
		{
		public:
			FunctionUnboundedException() :runtime_error("A function is not bounded in the queried interval") {}
		};



		/**
		 *   An abstract function with limits. f:DT->CT
		 */
		template<typename DT, typename CT>
		class ALFunction
		{
		private:
			CT cached_ymin, cached_ymax;
			bool cached_bounds_valid;

		public:
			/**
			 * reduce or increase the limit of the function, without changing y
			 */
			virtual void setLimits(DT lo, DT hi) = 0;
			/**
			 * query upper limit of the domain
			 */
			virtual DT limitHi() const = 0;
			/**
			 * lower limit of the domain
			 */
			virtual DT limitLo() const = 0;
			/**
			 * function evaluation returns y of codomain type CT for a value x of domain type DT
			 */
			virtual CT f(DT x) const = 0;
			/**
			 * create a copy of child class object - is used for function operations
			 */
			virtual ALFunction<DT, CT>* clone() = 0;
			/**
			 * create a new function object, which is the derivative function
			 */
			virtual ALFunction<DT, CT>* create_derivative() = 0;
			/**
			 * bound function values in the x-range defined by the hypercube between corner points lower left xmin and upper right xmax
			 */
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax) = 0;
		public:
			virtual ~ALFunction(){}
			ALFunction() :cached_bounds_valid(false) {}
			const CT operator() (DT x) const { return f(x); }
			// TODO this should be possible and would be better for correctness sake CT operator()(DT x) const { return f(x); }
			bool isInDomain(DT x){return limitLo()<=x && x<=limitHi();}
			/**
			 * a safe to use version of f(x), which enforces function bounds
			 */
			CT f_bounded(DT x) 
			{
				return f((std::max)(limitLo(),(std::min)(x,limitHi())));
			}
			/**
			 * evaluate function at multiple points provided by array xvec 
			 * @param xvec points in domain, which are evaluated, xvec length>= count
			 * @param yvec array for values of codomain, filled by function, length>= count
			 */
			virtual void f(DT* xvec, CT* yvec, unsigned int count) const //evaluate multi
			{
				for (unsigned int i = 0; i < count; i++)yvec[i] = f(xvec[i]);
			}
			/**
			 * bound function value on interval [ymin,ymax], using caching for bound computation
			 */
			void bound(CT& ymin, CT& ymax)
			{
				if (!cached_bounds_valid)
				{
					bound(limitLo(), limitHi(), cached_ymin, cached_ymax);
				}
				ymin = cached_ymin;
				ymax = cached_ymax;
			}
			/**
			 * clear the cache used in bound computation
			 */
			void invalidateCachedBounds()
			{
				cached_bounds_valid;
			}
		};

		/**
		 * compute function value in a given codomain dimension at multiple points x of the domain
		 * @param f the function to be evaluated
		 * @param x array of samples of the domain
		 * @param y array for query results
		 * @param dimension index of codomain-dimension
		 * @param count number of samples
		 */
		template<typename T, int N>
		void sample(ALFunction<T, adoreMatrix<T, N, 1>>* f, T* x, T* y, int dimension, unsigned int count)
		{
			adoreMatrix<T, N, 1> yi;
			for (unsigned int i = 0; i < count; i++)
			{
				yi = (*f)(x[i]);
				y[i] = yi(dimension);
			}
		}

		/**
		 * compute function value at multiple points x of the domain
		 * @param f the function to be evaluated
		 * @param x array of samples of the domain
		 * @param y matrix for query results
		 * @param K number of samples
		 * @param offset dimension shift in target array y
		 */
		template<typename T, int N>
		void sample(ALFunction<T, adoreMatrix<T, N, 1>>* f, const adoreMatrix<T,1,0>& x,adoreMatrix<T,N,0>& y,unsigned int K,int offset=0)
		{
			for (unsigned int i = 0; i < K; i++)
			{
				dlib::set_colm(y,i+offset) = (*f)(x(i));
			}
		}


		/**
		 * compute function value at multiple points x of the domain
		 * @param f the function to be evaluated
		 * @param x array of samples of the domain
		 * @param y array for query results
		 * @param count number of samples
		 */
		template<typename T, int N>
		void sample(ALFunction<T, adoreMatrix<T, N, 1>>* f, T* x, T* y, unsigned int count)//y must have size N*count
		{
			adoreMatrix<T, N, 1> yi;
			for (unsigned int i = 0; i < count; i++)
			{
				yi = (*f)(x[i]);
				for (unsigned int j = 0; j < N; j++)
				{
					y[i*N + j] = yi(j);
				}
			}
		}

		/**
		 *   An abstract function mapping from a scalar domain to a vector codomain, f:T->T^N
		 */
		template<typename T, int N>
		class AScalarToN : public ALFunction<T, adoreMatrix<T, N, 1>>
		{
		public:
			typedef T DT;
			typedef adoreMatrix<T, N, 1> CT;
			typedef ALFunction<DT, T> SUBFUN;
			/**
			 * evaluate multi, yvec must have at least size sizeof(T[N*count])
			 * ordering is y0(x[0]),y1(x[0]),y2(x[0]),...,yN(x[0]),y0(x[1]),...yN(x[1]),...,y0(x[count-1]),...,yN(x[count-1])
			 */
			void toArray(DT* xvec, T* yvec, unsigned int count)
			{
				for (unsigned int i = 0; i < count; i++)
				{
					for (unsigned int j = 0; j < count; j++)
					{
						yvec[i*N + j] = fi(xvec[i], j);
					}
				}
			}
			//sapmle a range of x values and evaluate for one row of the vector only
			void toArray(DT* xvec, T* yvec, unsigned int count, unsigned int row)
			{
				for (unsigned int i = 0; i < count; i++)
				{
					yvec[i] = fi(xvec[i], row);
				}
			}
			/**
			 * gives access to a scalar sub-function. does not create a new object, so use clone() to get your own instance of the subfunction.
			 */
			virtual SUBFUN* dimension(int i) = 0;
			/**
			 * scalar evaluation of function: for y-component dim
			 */
			virtual T fi(DT x, int dim) const = 0;
			/**
			 *  apply operation to function sub-dimensions: multiply with matrix of lower dimension in range rowi to rowj, with A.nc==A.nr==rowj-rowi+1
			 */
			virtual void multiply(adoreMatrix<T, 0, 0> A, int rowi, int rowj) = 0;
			/**
			 *  apply operation to function subdimensions: add a vector to rowi to rowj
			 */
			virtual void add(adoreMatrix<T, 0, 1> b, int rowi, int rowj) = 0;
			/**
			 *  short notations
			 */
			virtual void operator*=(adoreMatrix<T, N, N> A) { this->multiply(A, 1, N); }
			virtual void operator+=(adoreMatrix<T, N, 1> b) { this->add(b, 1, N); }
			virtual void operator-=(adoreMatrix<T, N, 1> b) { this->add(-b, 1, N); }
		};

		/**
		 * compute function value at multiple points x of the domain
		 * @param f the function to be evaluated
		 * @param x array of samples of the domain
		 * @param y matrix for query results
		 * @param K number of samples
		 * @param offset dimension shift in target array y
		 */
		template<typename T, int N>
		void sample(AScalarToN<T,N>* f, const adoreMatrix<T,1,0>& x,adoreMatrix<T,N,0>& y,unsigned int K,int offset=0)
		{
			for (unsigned int i = 0; i < K; i++)
			{
				dlib::set_colm(y,i+offset) = (*f)(x(i));
			}
		}

		/**
		 * LConstFun - a function with constant output
		 */
		template<typename DT, typename CT>
		class LConstFun :public ALFunction<DT, CT>
		{
		public:
			CT m_value;
			DT m_xlo,m_xhi;
			LConstFun( const CT& value, DT xlo,  DT xhi) :m_value(value) ,m_xlo(xlo),m_xhi(xhi){}
			virtual CT f(DT x)   const override { return m_value; }
			virtual DT limitHi() const override { return m_xhi; }
			virtual DT limitLo() const override { return m_xlo; }
			virtual void invertDirection() {}
			virtual void setLimits(DT lo, DT hi) {m_xlo = lo;m_xhi = hi;}
			virtual ALFunction<DT, CT>* clone()override { return new LConstFun<DT, CT>(m_value,m_xlo,m_xhi); }
			virtual ALFunction<DT, CT>* create_derivative()override { return new LConstFun<DT, CT>(m_value * 0,m_xlo,m_xhi); }
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override { ymin = m_value; ymax = m_value; }
		};

		/**
		 *  LLinearFunction - a plain old straight line between two points
		 */
		template<typename DT, typename CT>
		class LLinearFunction : public ALFunction<DT, CT>
		{
		private:
			DT m_x0, m_x1, m_xLo, m_xHi;
			CT m_y0, m_dydx;
		public:
			LLinearFunction(DT x0, DT x1, CT y0, CT dydx)
			{
				m_x0 = x0;
				m_x1 = x1;
				m_xLo = x0;
				m_xHi = x1;
				m_y0 = y0;
				m_dydx = dydx;
			}
			LLinearFunction(DT x0, DT x1, CT y0, CT dydx, DT xLo, DT xHi)
			{
				m_x0 = x0;
				m_x1 = x1;
				m_xLo = xLo;
				m_xHi = xHi;
				m_y0 = y0;
				m_dydx = dydx;
			}
			virtual ~LLinearFunction()
			{
			}
		public://methods inherited from ALFunction interface
			//function evaluation returns y of codomain type CT for a value x of domain type DT
			virtual CT f(DT x) const override
			{
				return m_y0 + m_dydx*(x - m_x0);
			}
			virtual DT limitHi() const override
			{
				return m_xHi;
			}
			virtual DT limitLo() const override
			{
				return m_xLo;
			}
			//reduce or increase the limit of the function
			virtual void setLimits(DT lo, DT hi) override
			{
				m_xLo = lo;
				m_xHi = hi;
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				return new LConstFun<DT, CT>(m_dydx,m_xLo,m_xHi);
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				CT y0 = f(xmin);
				CT y1 = f(xmax);
				ymin = (adore::mad::min)(y0, y1);
				ymax = (adore::mad::max)(y0, y1);
			}
			virtual ALFunction<DT, CT>* clone() override
			{
				return new LLinearFunction<DT,CT>(m_x0,m_x1,m_y0,m_dydx,m_xLo,m_xHi);
			}
		};

		/**
		 * a helper class for combining two functions by addition
		 */
		template<typename DT, typename CT>
		class FunctionCombination_Addition :public ALFunction<DT, CT>
		{
		public:
			ALFunction<DT, CT>* a;
			ALFunction<DT, CT>* b;
			//Objects *a and *b are henceforth managed by the FunctionCombination. Supply a clone as a and b if you want to retain control over the original objects.
			FunctionCombination_Addition(ALFunction<DT, CT>* a, ALFunction<DT, CT>* b)
			{
				this->a = a;
				this->b = b;
			}
			virtual ~FunctionCombination_Addition()
			{
				delete a;
				delete b;
			}
			virtual CT f(DT x)   const override { return a->f(x) + b->f(x); }
			virtual DT limitHi() const override { return (std::min)(a->limitHi(), b->limitHi()); }
			virtual DT limitLo() const override { return (std::max)(a->limitLo(), b->limitLo()); }
			virtual void setLimits(DT lo, DT hi)override { a->setLimits(lo, hi); b->setLimits(lo, hi); }
			virtual ALFunction<DT, CT>* clone()override { return new FunctionCombination_Addition(a->clone(), b->clone()); }
			virtual ALFunction<DT, CT>* create_derivative()override { return new FunctionCombination_Addition(a->create_derivative(), b->create_derivative()); }
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				CT y0min, y0max, y1min, y1max;
				a->bound(xmin, xmax, y0min, y0max);
				b->bound(xmin, xmax, y1min, y1max);
				ymin = y0min + y1min;
				ymax = y0max + y1max;
			}
		};

		/**
		 * a helper class for combining two functions by multiplication
		 */
		template<typename DT, typename CT, typename CTa, typename CTb>
		class FunctionCombination_Multiplication :public ALFunction<DT, CT>
		{
		public:
			ALFunction<DT, CTa>* a;
			ALFunction<DT, CTb>* b;
			//Objects *a and *b are henceforth managed by the FunctionCombination. Supply a clone as a and b if you want to retain control over the original objects.
			FunctionCombination_Multiplication(ALFunction<DT, CTa>* a, ALFunction<DT, CTb>* b)
			{
				this->a = a;
				this->b = b;
			}
			virtual ~FunctionCombination_Multiplication()
			{
				delete a;
				delete b;
			}
			virtual CT f(DT x)   const override { return a->f(x)*b->f(x); }
			virtual DT limitHi() const override { return (std::min)(a->limitHi(), b->limitHi()); }
			virtual DT limitLo() const override { return (std::max)(a->limitLo(), b->limitLo()); }
			virtual void setLimits(DT lo, DT hi)override { a->setLimits(lo, hi); b->setLimits(lo, hi); }
			virtual ALFunction<DT, CT>* clone()override { return new FunctionCombination_Multiplication<DT, CT, CTa, CTb>(a->clone(), b->clone()); }
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				return new FunctionCombination_Addition<DT, CT>(
					new FunctionCombination_Multiplication<DT, CT, CTa, CTb>(
						a->create_derivative(),
						b->clone()
						),
					new FunctionCombination_Multiplication<DT, CT, CTa, CTb>(
						a->clone(),
						b->create_derivative()
						)
					);
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				CTa y0min, y0max;
				CTb y1min, y1max;
				a->bound(xmin, xmax, y0min, y0max);
				b->bound(xmin, xmax, y1min, y1max);
				imultiply(y0min, y0max, y1min, y1max, ymin, ymax);
				//auto y0_int = create_interval(y0min,y0max);
				//auto y1_int = create_interval(y1min,y1max);
				//imultiply(a,b,ymin,ymax);
				//auto y_int = y0_int*y1_int;
				//ymin = lower_bound(y_int);
				//ymax = upper_bound(y_int);
			}
		};

		/**
		 * a helper class for combining two functions by matrix multiplication
		 */
		template<typename T,int N, int M, int K>
		class FunctionCombination_Multiplication_Matrix :public ALFunction<T, adoreMatrix<T,N,K>>
		{
		public:
			typedef T DT;
			typedef adoreMatrix<T,N,M> CTa;
			typedef adoreMatrix<T,M,K> CTb;
			typedef adoreMatrix<T,N,K> CT;
			ALFunction<DT, CTa>* a;
			ALFunction<DT, CTb>* b;
			//Objects *a and *b are henceforth managed by the FunctionCombination. Supply a clone as a and b if you want to retain control over the original objects.
			FunctionCombination_Multiplication_Matrix(ALFunction<DT, CTa>* a, ALFunction<DT, CTb>* b)
			{
				this->a = a;
				this->b = b;
			}
			virtual ~FunctionCombination_Multiplication_Matrix()
			{
				delete a;
				delete b;
			}
			virtual CT f(DT x)   const override { return a->f(x)*b->f(x); }
			virtual DT limitHi() const override { return (std::min)(a->limitHi(), b->limitHi()); }
			virtual DT limitLo() const override { return (std::max)(a->limitLo(), b->limitLo()); }
			virtual void setLimits(DT lo, DT hi)override { a->setLimits(lo, hi); b->setLimits(lo, hi); }
			virtual ALFunction<DT, CT>* clone()override { return new FunctionCombination_Multiplication_Matrix<T,N,M,K>(a->clone(), b->clone()); }
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				return new FunctionCombination_Addition<DT, CT>(
					new FunctionCombination_Multiplication_Matrix<T,N,M,K>(
						a->create_derivative(),
						b->clone()
						),
					new FunctionCombination_Multiplication_Matrix<T,N,M,K>(
						a->clone(),
						b->create_derivative()
						)
					);
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				CTa y0min, y0max;
				CTb y1min, y1max;
				a->bound(xmin, xmax, y0min, y0max);
				b->bound(xmin, xmax, y1min, y1max);
				imultiply<T,N,M,K>(y0min, y0max, y1min, y1max, ymin, ymax);
				//auto y0_int = create_interval(y0min,y0max);
				//auto y1_int = create_interval(y1min,y1max);
				//imultiply(a,b,ymin,ymax);
				//auto y_int = y0_int*y1_int;
				//ymin = lower_bound(y_int);
				//ymax = upper_bound(y_int);
			}
		};

		/**
		 * a helper class for combining one function with a constant value
		 */
		template<typename DT, typename CT, typename CTa, typename CTb>
		class FunctionCombination_MultiplicationConst :public ALFunction<DT, CT>
		{
		public:
			CTa a;
			ALFunction<DT, CTb>* b;
			//Object *b is henceforth managed by the FunctionCombination. Supply a clone as a if you want to retain control over the original objects.
			FunctionCombination_MultiplicationConst(CTa a, ALFunction<DT, CTb>* b)
			{
				this->a = a;
				this->b = b;
			}
			virtual ~FunctionCombination_MultiplicationConst()
			{
				delete b;
			}
			virtual CT f(DT x)   const override { return a*b->f(x); }
			virtual DT limitHi() const override { return b->limitHi(); }
			virtual DT limitLo() const override { return b->limitLo(); }
			virtual void setLimits(DT lo, DT hi)override { b->setLimits(lo, hi); }
			virtual ALFunction<DT, CT>* clone()override { return new FunctionCombination_MultiplicationConst<DT, CT, CTa, CTb>(a, b->clone()); }
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				return new FunctionCombination_MultiplicationConst<DT, CT, CTa, CTb>(
					a,
					b->create_derivative()
					);
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				CTa y0min, y0max;
				CTb y1min, y1max;
				y0min = a; y0max = a;
				b->bound(xmin, xmax, y1min, y1max);
				imultiply(y0min, y0max, y1min, y1max, ymin, ymax);
			}
		};

		/**
		 * a helper class for combining two functions by chaining
		 */
		template<typename DTa, typename CTa, typename DTb>
		class FunctionCombination_Chain :public ALFunction<DTb, CTa>
		{
		public:
			ALFunction<DTa, CTa>* a;
			ALFunction<DTb, DTa>* b;
			typedef DTb DT;
			typedef CTa CT;
			//Objects *a and *b are henceforth managed by the FunctionCombination. Supply a clone as a and b if you want to retain control over the original objects.
			FunctionCombination_Chain(ALFunction<DTa, CTa>* a, ALFunction<DTb, DTa>* b)
			{
				this->a = a;
				this->b = b;
			}
			virtual ~FunctionCombination_Chain()
			{
				delete a;
				delete b;
			}
			virtual CTa f(DTb x)  const override { return a->f(b->f(x)); }
			virtual DTb limitHi() const override { return b->limitHi(); }
			virtual DTb limitLo() const override { return b->limitLo(); }
			virtual void setLimits(DT lo, DT hi)override { b->setLimits(lo, hi); }
			virtual ALFunction<DT, CT>* clone()override { return new FunctionCombination_Chain(a->clone(), b->clone()); }
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				return new FunctionCombination_Multiplication<DT, CT, CTa, DTa>(
					new FunctionCombination_Chain<DTa,CTa,DTb>(a->create_derivative(),b->clone()),
					b->create_derivative()
					);
			}
			virtual void bound(const DTb& xmin, const DTb& xmax, CTa& ymin, CTa& ymax)override
			{
				DTa ybmin, ybmax;
				b->bound(xmin, xmax, ybmin, ybmax);
				a->bound(ybmin, ybmax, ymin, ymax);
			}
		};

		/**
		 * a helper class for combining a T^1->T^Na with a T^1->T^Nb function by stacking
		 */
		template<typename T, int Na, int Nb>
		class FunctionCombination_Stack :public ALFunction<T, adoreMatrix<T, Na + Nb, 1>>
		{
		public:
			typedef adoreMatrix<T, Na + Nb, 1> CT;
			typedef adoreMatrix<T, Na, 1> CTa;
			typedef adoreMatrix<T, Nb, 1> CTb;
			typedef T DT;
			ALFunction<T, CTa>* a;
			ALFunction<T, CTb>* b;
			//Objects *a and *b are henceforth managed by the FunctionCombination. Supply a clone as a and b if you want to retain control over the original objects.
			FunctionCombination_Stack(ALFunction<T, CTa>* a, ALFunction<T, CTb>* b)
			{
				this->a = a;
				this->b = b;
			}
			virtual ~FunctionCombination_Stack()
			{
				delete a;
				delete b;
			}
			virtual CT f(T x) const override
			{
				adoreMatrix<T, Na + Nb, 1> y;
				set_rowm(y, dlib::range(0, Na - 1)) = a->f(x);
				set_rowm(y, dlib::range(Na, Na + Nb - 1)) = b->f(x);
				return y;
			}
			virtual DT limitHi() const override { return (std::min)(a->limitHi(), b->limitHi()); }
			virtual DT limitLo() const override { return (std::max)(a->limitLo(), b->limitLo()); }
			virtual void setLimits(DT lo, DT hi)override { a->setLimits(lo, hi); b->setLimits(lo, hi); }
			virtual ALFunction<DT, CT>* clone()override
			{
				return new FunctionCombination_Stack(a->clone(), b->clone());
			}
			virtual ALFunction<DT, CT>* create_derivative() override
			{
				return new FunctionCombination_Stack(
					(ALFunction<T, CTa>*)a->create_derivative(),
					(ALFunction<T, CTb>*)b->create_derivative()
				);
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				CTa yamin, yamax;
				CTb ybmin, ybmax;
				a->bound(xmin, xmax, yamin, yamax);
				b->bound(xmin, xmax, ybmin, ybmax);
				set_rowm(ymin, dlib::range(0, Na - 1)) = yamin;
				set_rowm(ymax, dlib::range(0, Na - 1)) = yamax;
				set_rowm(ymin, dlib::range(Na, Na + Nb - 1)) = ybmin;
				set_rowm(ymax, dlib::range(Na, Na + Nb - 1)) = ybmax;
			}
		};

		/**
		 * a helper class for combining two 1->N functions
		 */
		template<typename T>
		class FunctionCombination_StackScalar :public AScalarToN<T, 2>
		{
		public:
			typedef typename AScalarToN<T,2>::DT DT;
			typedef typename AScalarToN<T,2>::CT CT;
			typedef ALFunction<T,T> SUBFUN;
			SUBFUN* a;
			SUBFUN* b;
			//Objects *a and *b are henceforth managed by the FunctionCombination. Supply a clone as a and b if you want to retain control over the original objects.
			FunctionCombination_StackScalar(ALFunction<T, T>* a, ALFunction<T, T>* b)
			{
				this->a = a;
				this->b = b;
			}
			virtual ~FunctionCombination_StackScalar()
			{
				delete a;
				delete b;
			}
			virtual adoreMatrix<T, 2, 1> f(T x) const override
			{
				adoreMatrix<T, 2, 1> y;
				y(0) = a->f(x);
				y(1) = b->f(x);
				return y;
			}
			virtual DT limitHi() const override { return (std::min)(a->limitHi(), b->limitHi()); }
			virtual DT limitLo() const override { return (std::max)(a->limitLo(), b->limitLo()); }
			virtual void setLimits(DT lo, DT hi)override { a->setLimits(hi, lo); b->setLimits(hi, lo); }
			virtual ALFunction<DT, CT>* clone()override
			{
				return new FunctionCombination_StackScalar(
					a->clone(),
					b->clone()
				);
			}
			virtual SUBFUN* dimension(int i) override
			{
				if (i == 0)return a;
				else return b;
			}
			virtual T fi(T x, int i) const override
			{
				if (i == 0)return a->f(x);
				else return b->f(x);
			}
			virtual void multiply(adoreMatrix<T, 0, 0> A, int rowi, int rowj)
			{
				throw FunctionNotImplemented();
			}
			virtual void add(adoreMatrix<T, 0, 1> c, int rowi, int rowj)
			{
				throw FunctionNotImplemented();
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				return new FunctionCombination_StackScalar(
					a->create_derivative(),
					b->create_derivative()
				);
			}
			virtual void bound(const T& xmin, const T& xmax, CT& ymin, CT& ymax)override
			{
				T yamin, yamax, ybmin, ybmax;
				a->bound(xmin, xmax, yamin, yamax);
				b->bound(xmin, xmax, ybmin, ybmax);
				ymin(0) = yamin;
				ymin(1) = ybmin;
				ymax(0) = yamax;
				ymax(1) = ybmax;
			}
		};

		namespace funop
		{
			/**
			 *  addition of two functions a,b:DT->CT
			 */
			template<typename DT, typename CT>
			ALFunction<DT, CT>* add(ALFunction<DT, CT>* a, ALFunction<DT, CT>* b)
			{
				return new FunctionCombination_Addition<DT, CT>(a, b);
			}

			/**
			 *  multiplication (and matrix multiplication) of two functions a:DT->CTa, b:DT->CTb. Make sure that CT is chosen so that a(x)*b(x) \in CT
			 */
			template<typename DT, typename CT, typename CTa, typename CTb>
			ALFunction<DT, CT>* multiply(ALFunction<DT, CTa>* a, ALFunction<DT, CTb>* b)
			{
				return new FunctionCombination_Multiplication<DT, CT, CTa, CTb>(a, b);
			}

			/**
			 *  matrix multiplication of two functions a:DT->CTa, b:DT->CTb. Make sure that CT is chosen so that a(x)*b(x) \in CT
			 */
			template<typename T, int N,int M,int K>
			ALFunction<T, adoreMatrix<T,N,K> >* mmultiply(ALFunction<T, adoreMatrix<T,N,M> >* a, ALFunction<T, adoreMatrix<T,M,K> >* b)
			{
				return new FunctionCombination_Multiplication_Matrix<T,N,M,K>(a, b);
			}

			/**
			 *  the negative of a function
			 */
			template<typename DT, typename CT, typename CTa, typename CTb>
			FunctionCombination_MultiplicationConst<DT, CT, CTa, CTb>* minus(ALFunction<DT, CTb>* b)
			{
				return new FunctionCombination_MultiplicationConst<DT, CT, CTa, CTb>((CTa)-1, b);
			}

			/**
			*  chaining of functions a: DTI->CT, b: DT->DTI  , f(x) = a(b(x)), f: DT->CT
			*/
			template<typename DTa, typename CTa, typename DTb>
			ALFunction<DTb, CTa>* chain(ALFunction<DTa, CTa>* a, ALFunction<DTb, DTa>* b)
			{
				return new FunctionCombination_Chain<DTa, CTa, DTb>(a, b);
			}

			/**
			 *  stack - row-wise combination of functions: a:T->T^Na, b:T->T^Nb => f:T->T^(Na+Nb)
			 */
			template<typename T, int Na, int Nb>
			ALFunction<T, adoreMatrix<T, Na + Nb, 1>>* stack(ALFunction<T, adoreMatrix<T, Na, 1>>* a, ALFunction<T, adoreMatrix<T, Nb, 1>>* b)
			{
				return new FunctionCombination_Stack<T, Na, Nb>(a, b);
			}
			template<typename T>
			AScalarToN<T, 2>* stack(ALFunction<T, T>* a, ALFunction<T, T>* b)
			{
				return new FunctionCombination_StackScalar<T>(a, b);
			}

			/**
			 *  stretch - relocate the function's output to a different interval of input values
			 *  if from > to, the direction of the function is inverted
			 */
			template<typename DT, typename CT>
			ALFunction<DT, CT>* stretch(ALFunction<DT, CT>* f, DT from, DT to)
			{
				if (from < to)
				{
					return chain<DT, CT, DT>(f, new LLinearFunction<DT, DT>(from, to, f->limitLo(), (f->limitHi() - f->limitLo()) / (to - from)));
				}
				else
				{
					return chain<DT, CT, DT>(f, new LLinearFunction<DT, DT>(to, from, f->limitHi(), (f->limitHi() - f->limitLo()) / (to - from)));
				}
			}
		}

		/**
		 * a helper class for computing 1/f(x)
		 */
		template<typename T>
		class FunctionModification_ReciprocalScalar :public ALFunction<T, T>
		{
		public:
			typedef T DT;
			typedef T CT;
			ALFunction<T, T>* m_divisor;
			FunctionModification_ReciprocalScalar(ALFunction<T, T>* divisor) :m_divisor(divisor) {}
			virtual void setLimits(DT lo, DT hi)override { m_divisor->setLimits(lo, hi); }
			virtual CT f(DT x)   const override { return ((T)1) / m_divisor->f(x); }
			virtual DT limitHi() const override { return m_divisor->limitHi(); }
			virtual DT limitLo() const override { return m_divisor->limitLo(); }
			virtual ALFunction<DT, CT>* clone() { return new FunctionModification_ReciprocalScalar(m_divisor->clone()); }
			virtual ALFunction<DT, CT>* create_derivative()
			{
				return funop::multiply<T, T, T, T>(
					funop::minus<T, T, T, T>(
						new FunctionModification_ReciprocalScalar<T>(
							funop::multiply<T, T, T, T>(m_divisor->clone(), m_divisor->clone())
							)
						),
					m_divisor->create_derivative()	//not to forget the chaining...
					);
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)
			{
				T ymin_divisor;
				T ymax_divisor;
				m_divisor->bound(xmin, xmax, ymin_divisor, ymax_divisor);
				if (ymin_divisor < 0 && 0 < ymax_divisor)
				{
					throw FunctionUnboundedException();
				}
				else
				{
					if (ymin_divisor > 0)
					{
						ymax = f(ymin_divisor);
						ymin = f(ymax_divisor);
					}
					else
					{
						ymax = f(ymax_divisor);
						ymin = f(ymin_divisor);
					}
				}
			}
		};

		namespace funop
		{
			template<typename T>
			ALFunction<T, T>* reciprocal(ALFunction<T, T>* divisor)
			{
				return new FunctionModification_ReciprocalScalar<T>(divisor);
			}
		}
	}
}