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
#include <algorithm>

namespace adore
{
	namespace mad
	{
		/**
		 * interval definition.
		 * Correct: Use type adoreMatrix<interval<T>,n,m> to represent interval matrices.
		 * Wrong: The type interval<adoreMatrix<T,n,m>> might compile, but multiplication is not handled correctly
		 */
		template<typename T>
		struct interval
		{
			T lb, ub;
			interval(T x0, T x1) :lb(adore::mad::min(x0, x1)), ub(adore::mad::max(x0, x1)) {}
			interval(T x) :lb(x), ub(x) {}
			interval() :lb((T)0), ub((T)0) {}

			void set(T lb, T ub) { this->lb = lb; this->ub = ub; }
			void setMinMax(T x0, T x1){lb = (adore::mad::min)(x0, x1); ub = (adore::mad::max)(x0, x1);}

			interval<T>& operator+=(const interval<T>& rhs) { lb += rhs.lb; ub += rhs.ub; return *this; }
			friend interval<T> operator+(interval<T> lhs, const interval<T>& rhs) { lhs += rhs; return lhs; }
			interval<T>& operator-=(const interval<T>& rhs) { lb -= rhs.lb; ub -= rhs.ub; return *this; }
			friend interval<T> operator-(interval<T> lhs, const interval<T>& rhs) { lhs -= rhs; return lhs; }
			interval<T>& operator*=(const interval<T>& rhs)
			{
				T a, b, c, d;
				a = lb*rhs.lb;
				b = lb*rhs.ub;
				c = ub*rhs.lb;
				d = ub*rhs.ub;
				lb = (std::min)((std::min)(a, b), (std::min)(c, d));
				ub = (std::max)((std::max)(a, b), (std::max)(c, d));
				return *this;
			}
			friend interval<T> operator*(interval<T> lhs, const interval<T>& rhs) { lhs *= rhs; return lhs; }
			interval<T> operator-()
			{
				return interval<T>(-ub, -lb);
			}
		};

		template<typename T>
		std::ostream& operator<<(std::ostream& os, const interval<T>& obj)
		{
			os << '[' << obj.lb << ';' << obj.ub << ']';
			return os;
		}

		template<typename T> inline bool operator< (const interval<T>& lhs, const interval<T>& rhs) { return lhs.ub < rhs.lb; }//left is below right
		template<typename T> inline bool operator> (const interval<T>& lhs, const interval<T>& rhs) { return rhs < lhs; }//left is above right
		template<typename T> inline bool operator<=(const interval<T>& lhs, const interval<T>& rhs) { return lhs.ub <= rhs.lb; }//left is smug below right
		template<typename T> inline bool operator>=(const interval<T>& lhs, const interval<T>& rhs) { return rhs <= lhs; }//left is smug above right
		template<typename T> inline bool operator!=(const interval<T>& lhs, const interval<T>& rhs) { return rhs < lhs || lhs < rhs; }//non-intersect
		template<typename T> inline bool operator==(const interval<T>& lhs, const interval<T>& rhs) { return lhs.lb == rhs.lb && lhs.ub == rhs.ub; }//equality

		typedef interval<double>	idouble;
		typedef interval<float>		ifloat;
		typedef interval<int>		iint;

		template<typename T, int N, int M>
		adoreMatrix<interval<T>, N, M> create_interval(const adoreMatrix<T, N, M>& lb, const adoreMatrix<T, N, M>& ub)
		{
			adoreMatrix<interval<T>, N, M> result;
			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j < M; j++)
				{
					result(i, j) = interval<T>(lb(i, j), ub(i, j));
				}
			}
			return result;
		}

		template<typename T>
		interval<T> create_interval(const T& lb, const T& ub)
		{
			return interval<T>(lb, ub);
		}

		template<typename T, int N, int M>
		adoreMatrix<T, N, M> lower_bound(const adoreMatrix<interval<T>, N, M>& data)
		{
			adoreMatrix<T, N, M> result;
			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j < M; j++)
				{
					result(i, j) = data(i, j).lb;
				}
			}
			return result;
		}

		template<typename T>
		T lower_bound(const interval<T>& iv)
		{
			return iv.lb;
		}

		template<typename T, int N, int M>
		adoreMatrix<T, N, M> upper_bound(const adoreMatrix<interval<T>, N, M>& data)
		{
			adoreMatrix<T, N, M> result;
			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j < M; j++)
				{
					result(i, j) = data(i, j).ub;
				}
			}
			return result;
		}

		template<typename T>
		T upper_bound(const interval<T>& iv)
		{
			return iv.ub;
		}

		/** 
		 * interval multiplication for NxM * MxK
		 */
		template<typename T, int N, int M, int K>
		inline void imultiply(const adoreMatrix<T, N, M>& lba, const adoreMatrix<T, N, M>& uba, const adoreMatrix<T, M, K>& lbb, const adoreMatrix<T, M, K>& ubb, adoreMatrix<T, N, K>& lb, adoreMatrix<T, N, K>& ub)
		{
			adoreMatrix<interval<T>, N, M> a = create_interval<T, N, M>(lba, uba);
			adoreMatrix<interval<T>, M, K> b = create_interval<T, M, K>(lbb, ubb);
			adoreMatrix<interval<T>, N, K> c = a*b;
			lb = lower_bound<T, N, K>(c);
			ub = upper_bound<T, N, K>(c);
		}

		/** 
		 * interval multiplication for Nx1 * double
		 */
		template<typename T, long N>
		inline void imultiply(const adoreMatrix<T, N, 1l>& lba, const adoreMatrix<T, N, 1l>& uba,  const double& lbb,const double& ubb, adoreMatrix<T, N, 1l>& lb, adoreMatrix<T, N, 1l>& ub)
		{
			adoreMatrix<interval<T>, N,1> a = create_interval<T, N, 1>(lba, uba);
			interval<double> b = create_interval<double>(lbb, ubb);
			adoreMatrix<interval<T>, N, 1> c = a*b;
			lb = lower_bound<T, N, 1>(c);
			ub = upper_bound<T, N, 1>(c);
		}

		/** 
		 * interval multiplication for NxM * MxK
		 */
		template<typename T, int N, int M, int K>
		inline void imultiply(const adoreMatrix<interval<T>, N, M>& a, const adoreMatrix<interval<T>, M, K>& b, adoreMatrix<T, N, K>& lb, adoreMatrix<T, N, K>& ub)
		{
			adoreMatrix<interval<T>, N, K> c = a*b;
			lb = lower_bound<T, N, K>(c);
			ub = upper_bound<T, N, K>(c);
		}

		/** 
		 * interval multiplication for double * double
		 */
		template<typename T>
		inline void imultiply(const interval<T>& a, const interval<T>& b, T& lb, T& ub)
		{
			interval<T> c = a*b;
			lb = lower_bound<T>(c);
			ub = upper_bound<T>(c);
		}

		/** 
		 * interval multiplication for double * double
		 */
		inline void imultiply(const double& lba, const double& uba, const double& lbb, const double& ubb, double& lb, double& ub)
		{
			interval<double> a = create_interval<double>(lba, uba);
			interval<double> b = create_interval<double>(lbb, ubb);
			interval<double> c = a*b;
			lb = lower_bound<double>(c);
			ub = upper_bound<double>(c);
		}


		/**
		 * interval sin
		 */
		template<typename T>
		interval<T> sin(interval<T> x)
		{
			if (x.ub - x.lb > 2.0*M_PI)return interval<T>(-1, 1);
			T lb = adore::mad::remainder<T>(x.lb + M_PI_2, 2.0*M_PI) - M_PI_2;
			T ub = x.ub - (x.lb - lb);

			//lb, ub in fourth and first quadrant? monotonous...
			bool lb_q41 = -M_PI_2 <= lb && lb <= M_PI_2;
			bool ub_q41 = -M_PI_2 <= ub && ub <= M_PI_2;
			bool ub_q23 = M_PI_2 <= lb && lb <= M_PI + M_PI_2;

			if (lb_q41 && ub_q41)return interval<T>((std::sin)(lb), (std::sin)(ub));//both in q14: monotonous here
			if (lb_q41)return interval<T>((std::min)((std::sin)(x.lb), (std::sin)(x.ub)), 1); //lb in q14 and ub in q23 -> +1 upper bound
			if (ub_q23)return interval<T>((std::sin)(ub), (std::sin)(lb));//both in q23: -monotonous here
			return interval<T>(-1, (std::max)((std::sin)(lb), (std::sin)(ub)));//lb in q23, ub in q45 -> -1 lower bound
		}

		/**
		 * interval cos
		 */
		template<typename T>
		interval<T> cos(interval<T> x)
		{
			return sin(x + interval<T>(M_PI_2));
		}

		/**
		 * interval atan2
		 */
		template<typename T>
		interval<T> atan2(interval<T> y, interval <T> x)
		{
			//0,0 included
			if (x.lb <= 0 && x.ub >= 0 && y.lb <= 0 && y.ub >= 0)return interval<T>(-M_PI, M_PI);
			//fully included in quadrants
			if (x.lb > 0 && y.lb >= 0) return interval<T>((std::atan2)(y.lb, x.ub), (std::atan2)(y.ub, x.lb));// I
			if (x.ub <= 0 && y.lb > 0) return interval<T>((std::atan2)(y.ub, x.ub), (std::atan2)(y.lb, x.lb));// II
			if (x.ub < 0 && y.ub <= 0) return interval<T>((std::atan2)(y.ub, x.lb), (std::atan2)(y.lb, x.ub));// III
			if (x.lb >= 0 && y.ub < 0) return interval<T>((std::atan2)(y.lb, x.lb), (std::atan2)(y.ub, x.ub));// IV
			//fully included in halfs
			if (x.lb > 0) return interval<T>((std::atan2)(y.lb, x.lb), (std::atan2)(y.ub, x.lb));// IV-I
			if (y.lb > 0) return interval<T>((std::atan2)(y.lb, x.ub), (std::atan2)(y.lb, x.lb));// I-II
			if (x.ub < 0) return interval<T>((std::atan2)(y.ub, x.ub), (std::atan2)(y.lb, x.ub) + M_PI*(T)2);// II-III (incl. PI->-PI transition)
			if (y.ub < 0) return interval<T>((std::atan2)(y.ub, x.lb), (std::atan2)(y.ub, x.ub));// III-IV
			return interval<T>(-M_PI, M_PI);
		}
	}
}