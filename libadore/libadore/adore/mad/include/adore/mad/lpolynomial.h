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

#include <algorithm>

namespace adore
{
	namespace mad
	{
		template<typename T, int N, int M>
		void poly_parameter_derivative(adoreMatrix<T, N, M + 1>& p);


		/**
		 * evaluate a polynomial \mathbb{R}->\mathbb{R}^N: y=p0 + p1 x +p2 x� + pM x^M, 
		 * given a parameter matrix data=[p0,p1,p2...] and a value x
		 */
		template<typename T, int N, int M>
		adoreMatrix<T, N, 1> evaluate_poly(const adoreMatrix<T, N, M + 1>& data, const T & x)
		{
			adoreMatrix<T, N, 1> y = dlib::colm(data, M);
			for (int i = M - 1; i >= 0; i--)
			{
				y = y * x;
				y = y + dlib::colm(data, i);
			}
			return y;
		}

		/**
		 *  given a set of parameters p of a polynomial y=p0 + p1 x +p2 x� +...,
		 *  create a set of parameters q for a polynomial y_s, so that y_s([c,d])=y([a,b]),
		 *  e.g. compute parameters for y_s(x)=y((x-c)/(d-c)*(b-a)+a)
		 *  task is also known as "polynomial shift"
		 */
		template<typename T, int N, int M>
		adoreMatrix<T, N, M + 1> stretch_poly_parameters(adoreMatrix<T, N, M + 1> data, const T& a, const T& b, const T& c, const T& d)
		{
			T s = (b - a) / (d - c);
			T si = 1;
			adoreMatrix<T, N, 1> y_tmp;
			adoreMatrix<T, 1, M + 1> ones_data = dlib::ones_matrix<T>(1, M + 1);
			adoreMatrix<T, N, M + 1> new_data = dlib::zeros_matrix<T>(N, M + 1);
			set_colm(new_data, 0) = evaluate_poly<T, N, M>(data, -c*s + a);
			for (int i = 1; i <= M; i++)
			{
				//s^i, has to be accounted for as d/dx y(xs-cs+a) = y^(i)(xs-cs+a)*s^i
				si *= s;
				//comparison of first coefficient of derivative at y_s^(i)(0):=y^(i)(-cs+a)
				poly_parameter_derivative<T, N, M>(data);										// p4 * x^4 --> 4 p4 * x^3
				poly_parameter_derivative<T, 1, M>(ones_data);									// 1  * x^4 --> 4 1  * x^3
				set_colm(new_data, i) = evaluate_poly<T, N, M>(data, -c*s + a) / ones_data(0) * si;  // evaluate the derivative i @ -cs+a, devide by factor in front of pi
			}
			return new_data;
		}
		/**
		 *  given a set of parameters p of a polynomial y=p0 + p1 x +p2 x� +...,
		 *  compute the parameters of the derivative y'=p1 + 2p2 x+ 3p3 + ...,
		 */
		template<typename T, int N, int M>
		void poly_parameter_derivative(adoreMatrix<T, N, M + 1>& p)
		{
			for (int i = 0; i < M; i++)
			{
				dlib::set_colm(p, i) = dlib::colm(p, i + 1) * (T)(i + 1);
			}
			dlib::set_colm(p, M) = dlib::zeros_matrix<T>(N, 1);//decrease order
		}

		/**
		 *  create polynomial parameters y=p0 + p1 x +p2 x� +... for a polynomial of degree M
		 *  which satisfy an initial condition vector c, with y^(i)(0)=c(i), with c sized (M+1)x1
		 *  (allowing to create a polynomial for an integrator chain)
		 */
		template<typename T, int M>
		adoreMatrix<T, 1, M + 1> poly_parameter_for_initial_condition(adoreMatrix<T, M + 1, 1> c)
		{
			adoreMatrix<T, 1, M + 1> ones_data = dlib::ones_matrix<T>(1, M + 1);
			adoreMatrix<T, 1, M + 1> new_data = dlib::zeros_matrix<T>(1, M + 1);
			new_data(0) = c(0);
			for (int i = 1; i <= M; i++)
			{
				poly_parameter_derivative<T, 1, M>(ones_data);
				new_data(i) = c(i) / ones_data(0);
			}
			return new_data;
		}

		/**
		 * compute bounds on polynomial according to Cargo, Shisha 1965: The Bernstein Form of a Polynomial
		 * first the polynomial is normalized to the interval x\in[0,1], then the bernstein form is used to bound y'([0,1])
		 */
		template<typename T, int N, int M>
		void bound_poly(const adoreMatrix<T, N, M + 1>& data, const T& x0, const T& x1, adoreMatrix<T, N, 1> & ymin, adoreMatrix<T, N, 1> & ymax)
		{
			adoreMatrix<T, N, M + 1> n_data = stretch_poly_parameters<T, N, M>(data, x0, x1, 0, 1);//always stretch to [0,1] interval
			ymin = colm(n_data, 0);
			ymax = ymin;
			adoreMatrix<T, N, 1> b;
			T k_choose_r; //k �ber r
			T M_choose_r; //M �ber r
			for (int k = 0; k <= M; k++)
			{
				b = dlib::zeros_matrix<T>(N, 1);
				for (int r = 0; r <= k; r++)
				{
					k_choose_r = (T)adore::mad::binomial(k, r);//k �ber r
					M_choose_r = (T)adore::mad::binomial(M, r);//M �ber r
					T c = k_choose_r / M_choose_r;
					for (int j = 0; j < N; j++)
					{
						b(j) += n_data(j, r)*c;
					}
				}
				ymin = adore::mad::min(ymin, b);
				ymax = adore::mad::max(ymax, b);
			}
		}

		/**
		 * LPolynomialS - a polynomial with scalar y value, mapping T->T
		 */
		template<typename T, int M>
		class LPolynomialS :public ALFunction<T, T>
		{
		private:
			typedef T DT;
			typedef T CT;
			adoreMatrix<T, 1, M + 1> m_data;
			DT m_xmin, m_xmax;
		public:
			LPolynomialS(const adoreMatrix<T, 1, M + 1>& data, DT xmin, DT xmax) :m_data(data), m_xmin(xmin), m_xmax(xmax) {}
		public:
			virtual CT f(DT x) const override
			{
				return evaluate_poly<T, 1, M>(m_data, x)(0);
			}
			virtual DT limitHi() const override
			{
				return m_xmax;
			}
			virtual DT limitLo() const override
			{
				return m_xmin;
			}
			virtual void setLimits(DT lo, DT hi)override
			{
				m_xmin = lo;
				m_xmax = hi;
			}
			virtual ALFunction<DT, CT>* clone()override
			{
				return new LPolynomialS(m_data, m_xmin, m_xmax);
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				if (M == 0)//already a constant polynomial -> return const zero function
				{
					return new LPolynomialS<T, M>(m_data*(T)0, m_xmin, m_xmax);
				}
				else
				{
					adoreMatrix<T, 1, (M < 0) ? 0 : M> new_data;
					new_data = dlib::colm(m_data, dlib::range(1, M));
					for (int i = 0; i < M; i++)
					{
						new_data(0, i) = new_data(0, i) * (T)(i + 1);
					}
					return new LPolynomialS<T, (M - 1 < 0) ? 0 : M - 1>(new_data, m_xmin, m_xmax);
				}
			}
			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax)override
			{
				adoreMatrix<T, 1, 1> ymin_tmp;
				adoreMatrix<T, 1, 1> ymax_tmp;
				bound_poly<T, 1, M>(m_data, xmin, xmax, ymin_tmp, ymax_tmp);
				ymin = ymin_tmp(0);
				ymax = ymax_tmp(0);
			}
		};

		/**
		 *	LPolynomialM - a polynomial with vector valued y: Mapping T->adoreMatrix<T,N,1>
		 */
		template<typename T, int N, int M>
		class LPolynomialM : public AScalarToN<T, N>
		{
		private:
			adoreMatrix<T, N, M + 1> m_data;//m_data is organized as [p_0,p_1,...,p_M] for a polynomial y=p_0 + x * p_1 + ... + x^M * p_M
			T m_xmin;
			T m_xmax;
		public:
			typedef adoreMatrix<T, N, 1> CT;
			typedef T DT;

			LPolynomialM(const adoreMatrix<T, N, M + 1>& data, T xmin, T xmax) :m_data(data), m_xmin(xmin), m_xmax(xmax)
			{
				for (int i = 0; i < N; i++)
				{
					single_dimensions[i] = OneDimension(this, i);
				}
			}

		public:// from ALFunction
			virtual CT f(DT x) const override
			{
				return evaluate_poly<T, N, M>(m_data, x);
			}
			virtual DT limitHi() const override
			{
				return m_xmax;
			}

			virtual DT limitLo() const override
			{
				return m_xmin;
			}

			virtual void setLimits(DT lo, DT hi) override
			{
				m_xmin = lo;
				m_xmax = hi;
			}
			virtual ALFunction<DT, CT>* clone()override
			{
				return new LPolynomialM<T, N, M>(m_data, m_xmin, m_xmax);
			}
			virtual ALFunction<DT, CT>* create_derivative()override
			{
				if (M <= 0)//already a constant polynomial -> return const zero function
				{
					return new LPolynomialM<T, N, M>(m_data*(T)0, m_xmin, m_xmax);
				}
				else
				{
					adoreMatrix<T, N, M> new_data;
					new_data = dlib::colm(m_data, dlib::range(1, M));
					for (int i = 0; i < M; i++)
					{
						dlib::set_colm(new_data, i) = dlib::colm(m_data, i + 1) * (T)(i + 1);
					}
					return new LPolynomialM<T, N, (M - 1 < 0) ? 0 : M - 1>(new_data, m_xmin, m_xmax);
				}
			}

			virtual void bound(const DT& xmin, const DT& xmax, CT& ymin, CT& ymax) override
			{
				bound_poly<T, N, M>(m_data, xmin, xmax, ymin, ymax);
			}

		private:
			class OneDimension :public ALFunction<DT, T>
			{
			private:
				LPolynomialM* m_parent;
				int m_row;
			public:
				OneDimension() {}
				OneDimension(LPolynomialM* parent, int row) :m_parent(parent), m_row(row) {}
				virtual T f(DT x)    const override { return m_parent->fi(x, m_row); }
				virtual DT limitHi() const override { return m_parent->limitHi(); }
				virtual DT limitLo() const override { return m_parent->limitLo(); }
				virtual void setLimits(DT lo, DT hi)override { throw FunctionNotImplemented(); }
				virtual ALFunction<DT, T>* clone()override { return new LPolynomialS<T, M>(dlib::rowm(m_parent->m_data, m_row), m_parent->m_xmin, m_parent->m_xmax); }
				virtual ALFunction<DT, T>* create_derivative()override
				{
					auto scalar = clone();
					auto der = scalar->create_derivative();
					delete scalar;
					return der;
				}
				virtual void bound(const DT& xmin, const DT& xmax, T& ymin, T& ymax) override
				{
					adoreMatrix<T, 1, 1> ymin_tmp;
					adoreMatrix<T, 1, 1> ymax_tmp;
					bound_poly<T, 1, M>(rowm(m_parent->m_data, m_row), xmin, xmax, ymin_tmp, ymax_tmp);
					ymin = ymin_tmp(0);
					ymax = ymax_tmp(0);
				}
			};
			OneDimension single_dimensions[N];
		public:// from AScalarToN
			typedef ALFunction<T, T> SUBFUN;
			virtual T fi(T x, int row) const override
			{
				return evaluate_poly<T, 1, M>(rowm(m_data, row), x)(0);
			}
			virtual SUBFUN* dimension(int i) override
			{
				return &single_dimensions[i];
			}
			virtual void multiply(adoreMatrix<T, 0, 0> A, int rowi, int rowj)override
			{
				dlib::set_subm(m_data, dlib::range(rowi, rowj),dlib::range(1,M)) = A *  dlib::subm(m_data, dlib::range(rowi, rowj),dlib::range(1,M));
			}
			virtual void add(adoreMatrix<T, 0, 1> b, int rowi, int rowj)override
			{
				dlib::set_subm(m_data, dlib::range(rowi, rowj), dlib::range(0, 0)) = subm(m_data, dlib::range(rowi, rowj), dlib::range(0, 0)) + b;
			}
		};
	}
}