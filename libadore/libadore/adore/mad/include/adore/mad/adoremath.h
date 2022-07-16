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
			
			result(0, n - 1) = (std::min)(x1 - eps, result(0, n - 1));///resolve rounding problem to not exceed limit
			return result;
		}
		/**
		 * fills an array with n values evenly spaced between x0 and x1
		 */
		template<typename T,typename Tarray>
		void linspace(T x0, T x1, Tarray& target, int n)
		{
			T eps = 1e-10;//there seems to be a problem with floating point precision in debug mode
			for (int i = 0; i < n; i++)
			{
				target[i] = x0 + (x1 - x0) / (T)(n - 1)*(T)i;
			}
			T tmp = target[n - 1];
			target[n - 1] = (std::min)(x1 - eps,tmp) ;///resolve rounding problem to not exceed limit
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
		 * @brief Transformation from Euclidean coordinate system to a relative coordinate system represented by linear-piecewise function xy and normal.
		 * The approximation of the originally non-linear function leads to inconsistencies with simpler methods.
		 * The following equation is fulfilled: q=pi+s/L*(pj-pi)+t*(ni+s/L*(nj-ni))
		 * @param qX the point to be transformed, X component
		 * @param qY the point to be transformed, Y component
		 * @param pi the initial point of the linearly approximated baseline (pis,pix,piy)
		 * @param pj the final point of the linearly approximated baseline (pjs,pjx,pjy)
		 * @param ni the initial normal vector (nix,niy)
		 * @param nj the final normal vector (njx,njy)
		 * @param s output: relative coordinate transversal component
		 * @param t output: relative coordinate lateral component
		 * @return true if transformation successful, false if qX,qY is not in the domain
		 */
		template<typename T1, typename T2>
		bool toRelativeWithNormalExtrapolation(double qX,double qY,const T1 pi,const T1 pj, const T2 ni, const T2 nj, double& s, double & t)
		{
			const double d = std::sqrt(  (pi(1)-pj(1))*(pi(1)-pj(1))  +  (pi(2)-pj(2))*(pi(2)-pj(2))  );
			if(d<1e-6)return false;//distance between points is too short

			const double qY_ni = -ni(1)*(qX-pi(1)) + ni(0)*(qY-pi(2));//hight of q above ni
			if(qY_ni>0.0)
			{
				//std::cout<<"q above ni, out of domain"<<std::endl;
				return false;//above ni->out of domain
			}
			const double qY_nj = -nj(1)*(qX-pj(1)) + nj(0)*(qY-pj(2));//hight of q above nj
			if(qY_nj<=0.0)
			{
				//std::cout<<"q below nj, out of domain"<<std::endl;
				return false;//below nj->out of domain
			}
			const double cross_n = ni(0)*nj(1)-ni(1)*nj(0);//angle between normals
			if(std::abs(cross_n)>1e-10)//if normals are not parallel
			{
				const double s_ni = (pj(1)*nj(1)-pj(2)*nj(0)+pi(2)*nj(0)-pi(1)*nj(1)) / cross_n;//distance of normal crossing along ni
				const double qX_ni =  ni(0)*(qX-pi(1)) + ni(1)*(qY-pi(2));//distance of q along ni
				if(s_ni<0.0)
				{
					if(qX_ni<s_ni)
					{
						//std::cout<<"q not in arc a)"<<std::endl;
						return false;//query point is not in arc/domain
					}
				}
				else
				{
					if(qX_ni>s_ni)
					{
						//std::cout<<"q not in arc b)"<<std::endl;
						return false;//query point is not in arc/domain
					}
				}
			}
			const double L = pj(0)-pi(0);
			const double cosv = (pj(1)-pi(1))/d;
			const double sinv = (pj(2)-pi(2))/d;
			const double qXr = cosv*(qX-pi(1)) + sinv*(qY-pi(2));
			const double qYr =-sinv*(qX-pi(1)) + cosv*(qY-pi(2));
			const double nixr = cosv*ni(0) + sinv*ni(1);
			const double niyr =-sinv*ni(0) + cosv*ni(1);
			const double njxr = cosv*nj(0) + sinv*nj(1);
			const double njyr =-sinv*nj(0) + cosv*nj(1);
			const double a = -d*niyr + d*njyr;
			const double b = niyr*qXr - njyr*qXr + d*niyr - nixr*qYr + njxr*qYr;
			const double c = -niyr*qXr + nixr*qYr;
			
			double sr=0;
			if(std::abs(a)>1e-10)
			{
				const double s1 = (-b+std::sqrt(b*b-4.0*a*c))*0.5/a;
				const double s2 = (-b-std::sqrt(b*b-4.0*a*c))*0.5/a;
				sr = (0.0<=s1 && s1<=1.0)?s1:s2;
				//std::cout<<"s1="<<s1<<std::endl;
				//std::cout<<"s2="<<s2<<std::endl;
			}
			else if(std::abs(b)>1e-10)
			{
				sr = -c/b;
			}
			else
			{
				return false;
			}
			if(!(0.0<=sr && sr<=1.0))
			{
				//std::cout<<"sr not in [0,1]"<<std::endl;
				return false;
			}
			

			s = pi(0) + sr * L;
			const double q0X = (1.0-sr) * pi(1) + sr * pj(1);
			const double q0Y = (1.0-sr) * pi(2) + sr * pj(2);
			const double dX = qX-q0X;
			const double dY = qY-q0Y;
			const double nx = (1.0-sr) * ni(0) + sr * nj(0);
			const double ny = (1.0-sr) * ni(1) + sr * nj(1);
			const double nL = std::sqrt(nx*nx+ny*ny);
			t = (nx * dX + ny * dY) / nL;
			const double e = (-ny * dX + nx * dY) / nL;
			if(std::abs(e)>1.0e-10)
			{
				//std::cout<<"result not on normal vector, e="<<e<<std::endl;
				return false;
			}

			//std::cout<<"niyr"<<niyr<<std::endl;
			//std::cout<<"njyr"<<njyr<<std::endl;
			//std::cout<<"a="<<a<<std::endl;
			//std::cout<<"b="<<b<<std::endl;
			//std::cout<<"c="<<c<<std::endl;
			//std::cout<<"sr="<<sr<<std::endl;
			//std::cout<<"t="<<t<<std::endl;
			return true;
		}


		template<typename T1, typename T2>
		bool toRelativeWithNormalExtrapolation(double qX,double qY,T1 centerline,T2 normals,double& s, double& t)
		{
			adoreMatrix<double,3,1> pi,pj;
			adoreMatrix<double,2,1> ni,nj;
			for(int i=0;i<centerline->getData().nc()-1;i++)
			{
				const int j = i+1;
				for(int d=0;d<3;d++)pi(d)=centerline->getData()(d,i);
				for(int d=0;d<3;d++)pj(d)=centerline->getData()(d,j);
				for(int d=1;d<3;d++)ni(d-1)=normals->getData()(d,i);
				for(int d=1;d<3;d++)nj(d-1)=normals->getData()(d,j);
				if(toRelativeWithNormalExtrapolation(qX,qY,pi,pj,ni,nj,s,t))return true;
			}
			return false;
			// double stest,ttest;
			// bool value_found = false;
			// for(int i=0;i<centerline->getData().nc()-1;i++)
			// {
			// 	const int j = i+1;
			// 	for(int d=0;d<3;d++)pi(d)=centerline->getData()(d,i);
			// 	for(int d=0;d<3;d++)pj(d)=centerline->getData()(d,j);
			// 	for(int d=1;d<3;d++)ni(d-1)=normals->getData()(d,i);
			// 	for(int d=1;d<3;d++)nj(d-1)=normals->getData()(d,j);
			// 	if(toRelativeWithNormalExtrapolation(qX,qY,pi,pj,ni,nj,stest,ttest))
			// 	{
			// 		if(!value_found||std::abs(ttest)<std::abs(t))
			// 		{
			// 			value_found = true;
			// 			s = stest;
			// 			t = ttest;
			// 		}
			// 	}
			// }
			// return value_found;
		}


		/**
		 * @brief Transform from relative coordinates to Euclidean coordinates.
		 * @param s relative coordinate transversal component
		 * @param t relative coordinate lateral component
		 * @param pi first point of baseline (si,xi,yi)
		 * @param pj last point of baseline (si,xi,yi)
		 * @param ni first normal of baseline (nxi,nyi)
		 * @param nj last normal of baseline (nxj,nyj)
		 * @param X output, Euclidean x component
		 * @param Y output, Euclidean y component
		 * @return true if in domain
		 */
		template<typename T1, typename T2>
		bool fromRelative(double s,double t,const T1 pi,const T1 pj, const T2 ni, const T2 nj, double& X, double & Y, double& Z)
		{
			// if(s<pi(0)||pj(0)<s)return false;
			const double L = pj(0)-pi(0);
			const double sr = (s-pi(0))/L;
			const double q0X = (1.0-sr) * pi(1) + sr * pj(1);
			const double q0Y = (1.0-sr) * pi(2) + sr * pj(2);
			const double q0Z = (1.0-sr) * pi(3) + sr * pj(3);
			const double nx = (1.0-sr) * ni(0) + sr * nj(0);
			const double ny = (1.0-sr) * ni(1) + sr * nj(1);
			const double nL = std::sqrt(nx*nx+ny*ny);
			X = q0X + nx * t / nL;
			Y = q0Y + ny * t / nL;
			Z = q0Z;
			return true;
		}

		/**
		 * @brief transform from relative coordinates for given centerline and normal functions
		 * @param s relative coordinate transversal component
		 * @param t relative coordinate lateral component
		 * @param centerline centerline function s->(X,Y,..) of type LLinearPiecewiseFunctionM*
		 * @param normals normal function s->(nx,ny) of type LLinearPiecewiseFunctionM*
		 */
		template<typename T1,typename T2>
		void fromRelative(double s,double t,T1 centerline,T2 normals,double& X,double& Y,double& Z)
		{
			adoreMatrix<double,3,1> pi,pj;
			adoreMatrix<double,2,1> ni,nj;
			int i = centerline->findIndex(s);
			int j = i+1;
			for(int d=0;d<3;d++)pi(d)=centerline->getData()(d,i);
			for(int d=0;d<3;d++)pj(d)=centerline->getData()(d,j);
			for(int d=1;d<3;d++)ni(d-1)=normals->getData()(d,i);
			for(int d=1;d<3;d++)nj(d-1)=normals->getData()(d,j);
			fromRelative(s,t,pi,pj,ni,nj,X,Y,Z);
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
		 * bound - bounding a value above and below
		 */
		template<typename T>
		inline void bound(T lb,T value[], size_t count, T ub)
		{
			for (size_t a=0; a< count;++a)
			{
				value[a] = bound(lb, value[a], ub);
			}
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
		template <typename T>
		inline int signum(T val)
		{
    		return (T(0) < val) - (val < T(0));
		}
	}
}