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

#include <dlib/matrix.h>
#define _USE_MATH_DEFINES //
#include <math.h>
#include <algorithm>

namespace adore
{
	namespace mad
	{
		//template<typename T,int R=0,int C=0>
		//class Matrix:public dlib::matrix<T,R,C>{};  --> doesnt work because of friend functions
		//typedef dlib::matrix Matrix;  --> typedef is not compatible with templates
		//template<typename T,int R, int C>
		//using Matrix=dlib::matrix<T,R,C>; //only C++11
		#define adoreMatrix dlib::matrix//replace this with C++11 solution

		using namespace dlib;

		/**
		 * initialize an array with data
		 */
		template<typename T>
		void set(T* data,T value,int size)
		{
			for(int i=0;i<size;i++)
			{
				data[i]=value;
			}
		}

		/**
		 * provides a vector filled with a sequence of values
		 */
		template<typename T>
		adoreMatrix<T, 1, 0> sequence(T x0, T dx, T xend)
		{
			int n = ceil((xend - x0) / dx) + 1;
			adoreMatrix<T, 1, 0> result(1, n);
			for (int i = 0; i < n; i++)
			{
				result(0, i) = x0 + dx*(T)i;
			}
			result(0, n - 1) = xend;
			return result;
		}
		/**
		 * fills an array with a sequence of values
		 */
		template<typename T>
		int sequence(T x0, T dx, T xend, T* target, int max_size)
		{
			int n = (std::min)(ceil((xend - x0) / dx) + 1, max_size);
			for (int i = 0; i < n; i++)
			{
				target[i] = x0 + dx*(T)i;
			}
			target[n - 1] = xend;
			return n;
		}
		/**
		 * fills an array with a sequence of values
		 */
		template<typename T>
		void sequence(T x0, T dx, T* target, int size)
		{
			for (int i = 0; i < size; i++)
			{
				target[i] = x0 + dx*(T)i;
			}
		}
		/**
		 * provides a statically sized vector with n values evenly spaced between x0 and x1
		 */
		template<typename T, int n>
		adoreMatrix<T, 1, n> linspace(T x0, T x1)
		{
			T eps = 1e-10;//there seems to be a problem with floating point precision in debug mode
			adoreMatrix<T, 1, n> result;
			for (int i = 0; i < n; i++)
			{
				result(0, i) = x0 + (x1 - x0) / (T)(n - 1)*(T)i;
			}
			result(0, n - 1) = (std::min)(x1 - eps, result(0, n - 1));///resolve rounding problem to not exceed limit
			return result;
		}
		/**
		 * provides a dynamically sized vector with n values evenly spaced between x0 and x1
		 */
		template<typename T>
		adoreMatrix<T, 1, 0> linspace(T x0, T x1, int n)
		{
			T eps = 1e-10;//there seems to be a problem with floating point precision in debug mode
			adoreMatrix<T, 1, 0> result;
			result = dlib::zeros_matrix<T>(1, n);
			for (int i = 0; i < n-1; i++)
			{
				result(0, i) = x0 + (x1 - x0) / (T)(n - 1)*(T)i;
			}
			result(0, n - 1) = x1;
			return result;
			//result(0, n - 1) = (std::min)(x1 - eps, result(0, n - 1));///resolve rounding problem to not exceed limit
		}
		/**
		 * fills an array with n values evenly spaced between x0 and x1
		 */
		template<typename T>
		void linspace(T x0, T x1, T* target, int n)
		{
			T eps = 1e-10;//there seems to be a problem with floating point precision in debug mode
			for (int i = 0; i < n; i++)
			{
				target[i] = x0 + (x1 - x0) / (T)(n - 1)*(T)i;
			}
			target[n - 1] = (std::min)(x1 - eps, target[n - 1]);///resolve rounding problem to not exceed limit
		}
		/**
		 * copy values from a matrix into an array
		 */
		template<typename T>
		void copyToArray(const adoreMatrix<T> &m, T* target)
		{
			for (int i = 0; i < m.nc()*m.nr(); i++)target[i] = m(i);
		}
		/**
		 * copy values from a matrix into an array
		 */
		template<typename T,long nr,long nc>
		void copyToArray(const adoreMatrix<T,nr,nc> &m, T* target)
		{
			for (int i = 0; i < m.nc()*m.nr(); i++)target[i] = m(i);
		}
		/**
		 * copy a single row from a matrix to an array
		 */
		template<typename T,long nr,long nc>
		void copyRowToArray(const adoreMatrix<T,nr,nc> &m, T* target,int col)
		{
			for (int i = 0; i < m.nc(); i++)target[i] = m(col,i);
		}
		/**
		 * n �ber k, n choose k, as given in https://stackoverflow.com/questions/15301885/calculate-value-of-n-choose-k
		 */
		inline int binomial(int n, int k)
		{
			if (k == 0)
			{
				return 1;
			}
			else
			{
				return (n*binomial(n - 1, k - 1)) / k;
			}
		}

		/**
		 *	computes the remainder of x/d
		 */
		template<typename T>
		T remainder(T x, T d)
		{
			return x - ((x >= 0) ? (T)1 : (T)-1) * (std::floor)((std::abs)(x) / d) * d;
		}

		/**
		 *	computes the 2 norm of a vector x
		 */
		template<typename T, long N>
		T norm2(const adoreMatrix<T, N, 1>& x)
		{
			T qvalue = 0;
			for (int i = 0; i < x.nr(); i++)qvalue += x(i)*x(i);
			return (std::sqrt)(qvalue);
		}

		/**
		 * createAngularContinuity - correct jumps from +pi+\epsilon to -pi+\epsilon 
		 * If a linear piecewise representation of angles is interpolated, these jumps can lead to almost any result on the circle.
		 * To correct, angles are counted further than pi.
		 * Corrections are applied to data(row,:)
		 */
		template<typename T,long N>
		void createAngularContinuity(adoreMatrix<T,N,0>& data,int row)
		{
			T twopi = M_PI*2.0;
			T offset = 0.0;
			for(int j=1;j<data.nc();j++)
			{
				T di = data(row,j-1);
				T dj = data(row,j) + offset;
				T delta = (std::abs)(dj-di);
				if((std::abs)(dj+twopi-di)<delta)
				{
					dj += twopi;
					offset += twopi;
				}else if((std::abs)(dj-twopi-di)<delta)
				{
					dj -= twopi;
					offset -= twopi;
				}
				data(row,j) = dj;
			}
		}

		/**
		 *   computes the position of a point relative to a line
		 *	 line (a,b)->(c,d), point (e,f)
		 *   d_tangential is in [0,1] if point is projected inside line, is >1 if point is in front and <0 if point is behind
		 *	 d_normal is >0 if point is left of line and <0 if right of line. d_normal is also the normal distance
		 */
		template<typename T>
		inline void comparePointWithLine(T a, T b, T c, T d, T e, T f, T& d_tangential, T& d_normal)
		{
			//t=(c-a,d-b)
			//n=(-(d-b),c-a)
			//x=(e,f)
			T x = c - a;
			T y = d - b;
			T p = e - a;
			T q = f - b;
			T dd = x*x + y*y;
			d_tangential = (x*p + y*q) / dd;
			d_normal = (-y*p + x*q) / sqrt(dd);
		}
		/**
		 *   computes the longitudinal and lateral position wrt a line
		 *	 line (a,b)->(c,d), point (e,f)
		 */
		template<typename T>
		inline void getRelativeCoordinatesPointVSLine(T a, T b, T c, T d, T e, T f, T& d_tangential, T& d_normal)
		{
			//t=(c-a,d-b)
			//n=(-(d-b),c-a)
			//x=(e,f)
			T x = c - a;
			T y = d - b;
			T p = e - a;
			T q = f - b;
			T l = sqrt(x*x + y*y);
			d_tangential = (x*p + y*q) / l;
			d_normal = (-y*p + x*q) / l;
		}
		/**
		 *   computes the distance between a line and a point and saves the relative progress in rel
		 *	 line (a,b)->(c,d), point (e,f)
		 */
		template<typename T>
		inline double getDistancePointToLine(T a, T b, T c, T d, T e, T f, T& rel,T& n)
		{
			//t=(c-a,d-b)
			//n=(-(d-b),c-a)
			//x=(e,f)
			T x = c - a;
			T y = d - b;
			T p = e - a;
			T q = f - b;
			T l = sqrt(x*x + y*y);
			T t = (x*p + y*q) / l;
			n = (-y*p + x*q) / l;
			if(t<(T)0)//in front of line: return distance to a,b
			{
				rel = 0;
				return sqrt(p*p+q*q);
			}
			else
			{
				if(t>l)//in rear of line: return distance to c,d
				{
					p = e-c;
					q = f-d;
					rel = 1;
					return sqrt(p*p+q*q);
				}
				else //besides line: return normal distance
				{
					rel = t/l;
					return std::abs(n);
				}
			}
		}
		template<typename T>
		inline double getDistancePointToLine(T a, T b, T c, T d, T e, T f, T& rel)
		{
			T n;
			return getDistancePointToLine(a,b,c,d,e,f,rel,n);
		}


		/**
		 *	(a,b)->(c,d); (e,f)->(g,h)
		 *  returns true, if the lines are not parallel
		 *	sets x0 and x1 to the parameter of the intersection point, ip = (a,b)+x0*(c,d-a,b)
		 *	sets x0_inside to true, if the intersection point is between (a,b) and (c,d)
		 *	sets x1_inside to true, if the intersection point is between (e,f) and (g,h)
		 */
		template<typename T>
		inline bool intersectLines(T a, T b, T c, T d, T e, T f, T g, T h, T& x0, T& x1, bool& x0_inside, bool& x1_inside)
		{
			T p = c - a;
			T q = d - b;
			T r = g - e;
			T s = h - f;
			T w = (r*q - s*p);
			x0_inside = false;
			x1_inside = false;
			if (w == (T)0)return false;
			x1 = (a*q - b*p - e*q + f*p) / w;
			x0 = (e*s - f*r - a*s + b*r) / (-w);
			x0_inside = (T)0 <= x0 && x0 <= (T)1;
			x1_inside = (T)0 <= x1 && x1 <= (T)1;
			return true;
		}
		/**
		 *	(a,b)->(c,d); (e,f)->(g,h)
		 *  returns true, if the lines intersect: true crossing>eps, not only touching
		 *	sets x0 and x1 to the parameter of the intersection point, ip = (a,b)+x0*(c,d-a,b)
		 *	sets x0_inside to true, if the intersection point is between (a,b) and (c,d)
		 *	sets x1_inside to true, if the intersection point is between (e,f) and (g,h)
		 */
		template<typename T>
		inline bool intersectLines2(T a, T b, T c, T d, T e, T f, T g, T h, T& x0, T& x1, T eps)
		{
			bool x0_inside;
			bool x1_inside;
			T p = c - a;
			T q = d - b;
			T r = g - e;
			T s = h - f;
			T w = (r*q - s*p);
			x0_inside = false;
			x1_inside = false;
			if (w == (T)0)return false;
			x1 = (a*q - b*p - e*q + f*p) / w;
			x0 = (e*s - f*r - a*s + b*r) / (-w);
			x0_inside = (T)0+eps < x0 && x0 <-eps+(T)1;
			x1_inside = (T)0+eps < x1 && x1 <-eps+(T)1;
			return x0_inside && x1_inside;
		}

		/**
		 * bound - bounding a value above and below
		 */
		template<typename T>
		inline T bound(T lb,T value,T ub)
		{
			return (std::max)(lb,(std::min)(value,ub));
		}


		/**
		 * annotatedDataOrdering_fct - helps to sort double-annotated data of type std::pair(double,T) with std::sort
		 */
		template<typename T>
		bool annotatedDataOrdering_fct (std::pair<double,T> i,std::pair<double,T> j) { return (i.first<j.first); }



		/**
		 *	cross product for c-arrays: s:=u x v
		 */
		template<typename T>
		T* cross(T* u,T* v,T* s)
		{
			s[0]=u[1]*v[2]-u[2]*v[1];
			s[1]=u[2]*v[0]-u[0]*v[2];
			s[2]=u[0]*v[1]-u[1]*v[0];
			return s;
		}

		/**
		 * dot product for c-arrays: val:=u'*v
		 */
		template<int d,typename T>
		T dot(T* u,T* v)
		{
			T val=(T)0;
			for(int i=0;i<d;i++)
			{
				val+=u[i]*v[i];
			}
			return val;
		}



		/**
		 *	normalize length of a vector
		 */
		template<int k,typename T>
		T* normalize(T* v)
		{
			T l = (T)0;
			for(int i=0;i<k;i++)
			{
				l += v[i]*v[i];
			}
			l=((T)1)/std::sqrt(l);
			for(int i=0;i<k;i++)
			{
				v[i] *= l;
			}
			return v;
		}

		/**
		 * extend min and max to contain value
		 */
		template<typename T>
		void extendBounds(T& min, T value, T& max)
		{
			min=std::min(min,value);
			max=std::max(max,value);
		}
		/**
		 * test whether two intervals overlap
		 */
		template<typename T>
		bool overlaps(const T& a0, const T&a1,const T& b0, const T& b1)
		{
			return (b0<=a0 && a0<=b1) || (b0<=a1 && a1<=b1) || (a0<=b0 && b0<=a1) || (a0<=b1 && b1<=a1);
		}
		/**
		 * the minimum of four values
		 */
		template<typename T>
		T min(T a,T b,T c,T d)
		{
			return std::min(std::min(a,b),std::min(c,d));
		}
		/**
		 * min applied to two vectors
		 */
		template<typename T, long N, long M>
		adoreMatrix<T, N, M> min(adoreMatrix<T, N, M> a, const adoreMatrix<T, N, M>& b)
		{
			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j < M; j++)
				{
					a(i, j) = (std::min)(a(i, j), b(i, j));
				}
			}
			return a;
		}
		/**
		 * max applied to two vectors
		 */
		template<typename T, long N, long M>
		adoreMatrix<T, N, M> max(adoreMatrix<T, N, M> a, const adoreMatrix<T, N, M>& b)
		{
			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j < M; j++)
				{
					a(i, j) = (std::max)(a(i, j), b(i, j));
				}
			}
			return a;
		}

		inline double min( double a,const double & b)
		{
			return (std::min)(a, b);
		}
		inline double max( double a,const double & b)
		{
			return (std::max)(a, b);
		}
		inline float min( float a,const float & b)
		{
			return (std::min)(a, b);
		}
		inline float max( float a,const float & b)
		{
			return (std::max)(a, b);
		}
		inline int min( int a,const int & b)
		{
			return (std::min)(a, b);
		}
		inline int max( int a,const int & b)
		{
			return (std::max)(a, b);
		}
	}
}