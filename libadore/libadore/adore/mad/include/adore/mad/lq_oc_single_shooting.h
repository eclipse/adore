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
#include <adore/mad/fun_essentials.h>
#include <vector>
#include <qpOASES.hpp>

namespace adore
{
	namespace mad
	{
		/**
		 * define discrete time system matrix for an integrator chain
		 * @param Ad discrete time system matrix, result
		 * @param Bd discrete time input matrix, result
		 * @param T time step
		 */
		template<typename T,int d>//degree
		void define_integrator_chain(dlib::matrix<T,d,d>& Ad,dlib::matrix<T,d,1>& Bd,T dt)
		{
			Ad = dlib::zeros_matrix(Ad);
			T q[d+1];			q[0]=(T)1;
			T pow_dt[d+1]; pow_dt[0]=(T)1;
			for(int i=1;i<=d;i++)
			{
				q[i] = q[i-1]/(T)i;
				pow_dt[i] = pow_dt[i-1] * dt;
			}
			for(int i=0;i<d;i++)
			{
				for(int j=i;j<d;j++)
				{
					Ad(i,j) = q[j-i]*pow_dt[j-i];
				}
				Bd(i) = q[d-i]*pow_dt[d-i];
			}		
		}


		/**
		 * LQ_OC_single_shooting - solves finite horizon linear quadratic optimal control problem with qpOASES and single shooting approach.
		 *	The optimization variable [u0,u1,u2,...,u(k-1), eps] is used, with eps the slack in x dimensions used for soft-constraints.
		*  J= 1/2 (x-y)^T H(wx) (x-y) + 1/2 u^T H(wu) u + 1/2 eps^T H(weps) eps + eps^T geps
		*	 N is the number of states, R is the number of inputs, K is the number of time steps during optimization and P is the number of interpolation steps.
		*/
		template<int N, int R, int K,int P>//real_t=float/double,#x=N,#u=R,#T=K,#Tint=K*P
		class LQ_OC_single_shooting
		{
		public:
			typedef qpOASES::real_t real_t;
			static const int nV = R*K + N;///number of variables in the QP:   var=[u;eps]
			static const int nC = 2*N*K;///number of linear system constraints in QP: upper and lower bounds for x and u separately due to slack
			typedef dlib::matrix<real_t,N,N> t_Ad;///discrete time system matrix future version: one Ad,Bd per time step
			typedef dlib::matrix<real_t,N,R> t_Bd;///discrete time input matrix
			typedef adore::mad::LLinearPiecewiseFunctionM<double,N+R> t_resultfun;///function for result interpolation
			//weights for cost function
			typedef dlib::matrix<real_t,N,1> t_wx;///weights for states (will be put on main diagonal of Hx)
			typedef dlib::matrix<real_t,N,1> t_wx_end;///weights for states at endpoint
			typedef dlib::matrix<real_t,R,1> t_wu;///weights for inputs (will be put on main diagonal of Hu)
			typedef dlib::matrix<real_t,R,1> t_wu_end;///weights for inputs at endpoint
			typedef dlib::matrix<real_t,N,1> t_weps;///quadratic weights for slack (will be put in Heps) 
			typedef dlib::matrix<real_t,N,1> t_geps;///linear weights for slack
			//online data: bounds
			typedef dlib::matrix<real_t,N,K> t_lbx;///lower bound on state trace
			typedef dlib::matrix<real_t,N,K> t_ubx;///upper bound on state trace
			typedef dlib::matrix<real_t,R,K> t_lbu;///lower bound on input trace
			typedef dlib::matrix<real_t,R,K> t_ubu;///upper bound on input trace
			typedef dlib::matrix<real_t,R,K> t_lbu_hard;///lower bound on input trace, hard constraint going to lb
			typedef dlib::matrix<real_t,R,K> t_ubu_hard;///upper bound on input trace, hard constraint going to ub
			typedef dlib::matrix<real_t,N,1> t_ubeps;///lbeps==0, softconstraints for x and/or u if ubeps>0
			//state and reference
			typedef dlib::matrix<real_t,N,1> t_x0;///initial state
			typedef dlib::matrix<real_t,N,K> t_y;///desired states (reference) y=[y_0(t_1);y_1(t_1);y_2(t_1);y_0(t_2);y_1(t_2);y_2(t_2);...;y_0(t_K);y_1(t_K);y_2(t_K)] (for N=3)
			typedef dlib::matrix<real_t,N*K,1> t_y_flat;///desired states (reference) y=[y_0(t_1);y_1(t_1);y_2(t_1);y_0(t_2);y_1(t_2);y_2(t_2);...;y_0(t_K);y_1(t_K);y_2(t_K)] (for N=3)
			typedef dlib::matrix<real_t,R,K> t_uset;///set point
			typedef dlib::matrix<real_t,R*K,1> t_uset_flat;///set point
			//output
			typedef dlib::matrix<real_t,R,K> t_U;///the computed control [u(t_0),...,u(t_K-1)]
			typedef dlib::matrix<real_t,R*K,1> t_U_flat;///the computed control [u(t_0),...,u(t_K-1)]
			typedef dlib::matrix<real_t,N,K+1> t_X;///the computed states [x(t_1),..., x(t_K)]
			typedef dlib::matrix<real_t,N*(K+1),1> t_X_flat;///the computed states [x(t_1),..., x(t_K)]
			typedef dlib::matrix<real_t,N,1> t_eps;///the slack

		private://state of this object
			bool m_system_changed;
		private://data, which must be provided by user
			//discrete time system matrices
			t_Ad m_Ad;///discrete time system matrix future version: one Ad,Bd per time step
			t_Bd m_Bd;///discrete time input matrix
			t_Ad m_Ad_p;///system matrix for interpolation
			t_Bd m_Bd_p;///input matrix for interpolation
			std::vector<t_Ad*> m_Ad_powers;///powers of Ad, so these don't have to be recomputed online
			t_resultfun m_resultfun;
			//weights for cost function
			t_wx m_wx;///weights for states (will be put on main diagonal of Hx)
			t_wx_end m_wx_end;///weights for states at endpoint
			t_wu m_wu;///weights for inputs (will be put on main diagonal of Hu)
			t_wu_end m_wu_end;///weights for inputs at endpoint
			t_weps m_weps;///quadratic weights for slack (will be put in Heps) 
			t_geps m_geps;///linear weights for slack
			//online data: bounds
			t_lbx m_lbx;///lower bound on state trace
			t_ubx m_ubx;///upper bound on state trace
			t_lbu_hard m_lbu_hard;///lower bound on input trace, hard constraint going to lb
			t_ubu_hard m_ubu_hard;///upper bound on input trace, hard constraint going to ub
			t_ubeps m_ubeps;///lbeps==0, softconstraints for x and/or u if ubeps>0
			//online data: x0, y
			t_x0 m_x0;///initial state
			t_y m_y;///desired states (reference) y=[y_0(t_1);y_1(t_1);y_2(t_1);y_0(t_2);y_1(t_2);y_2(t_2);...;y_0(t_K);y_1(t_K);y_2(t_K)] (for N=3)
			t_y_flat m_y_flat;///desired states (reference) y=[y_0(t_1);y_1(t_1);y_2(t_1);y_0(t_2);y_1(t_2);y_2(t_2);...;y_0(t_K);y_1(t_K);y_2(t_K)] (for N=3)
			//output
			t_U m_U;///the computed control [u(t_0),...,u(t_K-1)]
			t_U_flat m_U_flat;
			t_X m_X;///the computed states [x(t_1),..., x(t_K)]
			t_X_flat m_X_flat;
			t_eps m_eps;
			t_uset m_uset;///set point
			t_uset_flat m_uset_flat;///set point
			
		private://intermediate matrices
			dlib::matrix<real_t,N*K,1> m_E;/// [pow(Ad,1);pow(Ad,2);...;pow(Ad,k)]*x0
			dlib::matrix<real_t,N*K,R*K> m_F;/// [B,0,0,..0;AB,B,0,...0;AAB,AB,B,...,0;...;pow(A,k-1)B,pow(A,k-2)B,pow(A,k-3),B,...,B]
			dlib::matrix<real_t,N*K,N*K> m_Hx;
			dlib::matrix<real_t,R*K,R*K> m_Hu;
			dlib::matrix<real_t,N,N> m_Heps;
			dlib::matrix<real_t,R*K,1> m_g1;///first part of overall linear cost term, g=[F^T Hx E - F^T Hx y; geps]=[g1;geps]
			dlib::matrix<real_t,R*K,R*K> m_H11;///upper left part of overal quadratic cost term H=[Hu + F^T Hx F, 0; 0, Heps]=[H11,0;0,Heps]

			dlib::matrix<real_t,N*K,nV> m_A1;///state soft constraints equation upper bound
			dlib::matrix<real_t,N*K,nV> m_A2;///state soft constraints equation lower bound
		private://variables for qpOASES interfacing
			qpOASES::QProblem* m_qproblem;
			real_t m_qpH[nV*nV];
			real_t m_qpg[nV];
			real_t m_qpA[nC*nV];
			real_t m_qplb[nV];
			real_t m_qpub[nV];
			real_t m_qplbA[nC];
			real_t m_qpubA[nC];
			real_t m_qpxOpt[nV];
			qpOASES::int_t m_qpnWSR_in;///qpOASES: "The integer argument nWSR species the maximum number of working set recalculations to be performed during the initial homotopy"
			qpOASES::int_t m_qpnWSR_out;///qpOASES: "The integer argument nWSR species the maximum number of working set recalculations to be performed during the initial homotopy"
			real_t m_qpcputime_in;///qpOASES: "If cputime is not the null pointer, it contains the maximum allowed CPU time in seconds for the whole initialisation (and the actually required one on output)."
			real_t m_qpcputime_out;///qpOASES: "If cputime is not the null pointer, it contains the maximum allowed CPU time in seconds for the whole initialisation (and the actually required one on output)."
		private://methods for intermediate matrix setup
			void initialize_userdata_matrices()
			{
				m_Ad = dlib::identity_matrix<real_t>(N);
				m_Bd = dlib::zeros_matrix(m_Bd);
				m_wx = dlib::ones_matrix(m_wx);
				m_wu = dlib::ones_matrix(m_wu);
				m_wx_end = dlib::ones_matrix(m_wx_end);
				m_wu_end = dlib::ones_matrix(m_wu_end);
				m_weps = dlib::zeros_matrix(m_weps);
				m_geps = dlib::ones_matrix(m_geps);
				m_lbx = -dlib::ones_matrix(m_lbx);
				m_ubx = dlib::ones_matrix(m_ubx);
				m_lbu_hard = -dlib::ones_matrix(m_lbu_hard);
				m_ubu_hard = dlib::ones_matrix(m_ubu_hard);
				m_ubeps = ((real_t)1e-6) * dlib::ones_matrix(m_ubeps);
				m_x0 = dlib::zeros_matrix(m_x0);
				m_y = dlib::zeros_matrix(m_y);
				m_X = dlib::zeros_matrix(m_X);
				m_X_flat = dlib::zeros_matrix(m_X_flat);
				m_U = dlib::zeros_matrix(m_U);
				m_U_flat = dlib::zeros_matrix(m_U_flat);
				m_uset = dlib::zeros_matrix(m_uset);
				m_uset_flat = dlib::zeros_matrix(m_uset_flat);
			}
			void initialize_memory()
			{
				m_Ad_powers.clear();
				for(int i=0;i<=K;i++)
				{
					auto M = new dlib::matrix<real_t,N,N>();
					*M = dlib::identity_matrix<real_t>(N);
					m_Ad_powers.push_back(M);
				}
				m_qproblem = new qpOASES::QProblem(nV,nC);
			}
			void delete_memory()
			{
				for(auto it = m_Ad_powers.begin();it!=m_Ad_powers.end();it++)
				{
					delete *it;
				}
				m_Ad_powers.clear();
				delete m_qproblem;
			}
			void initialize_interface_variables()
			{
				for(int i=0;i<nV*nV;i++)m_qpH[i]=(real_t)0.0;
				for(int i=0;i<nC*nV;i++)m_qpA[i]=(real_t)0.0;
				for(int i=0;i<nV;i++)m_qplb[i]=(real_t)0.0;
			}

			/**
			 *	initialize intermediate matrices with constants 0,1,-1, especially parts which are constant and not modified by updates
			*/
			void initialize_intermediate_matrices()
			{
				m_F = dlib::zeros_matrix(m_F);
				m_Hx = dlib::zeros_matrix(m_Hx);
				m_Hu = dlib::zeros_matrix(m_Hu);
				m_Heps = dlib::zeros_matrix(m_Heps);
				/**
				 *	A1 := [F  [-I;-I;-I;...-I]  0]	--> x upper bound soft constraint
				*/
				m_A1 = dlib::zeros_matrix(m_A1);
				for(int i=0;i<K;i++)
				{
					for(int j=0;j<N;j++)
					{
						m_A1(i*N+j,R*K + j) = (real_t)-1;
					}
				}
				/**
				 *	A2 := [F  [I;I;I;...I]  0]	--> x lower bound soft constraint
				*/
				m_A2 = dlib::zeros_matrix(m_A2);
				for(int i=0;i<K;i++)
				{
					for(int j=0;j<N;j++)
					{
						m_A2(i*N+j,R*K + j) = (real_t)1;
					}
				}

			}

			void updateAd()
			{
				for(int i=1;i<=K;i++)
				{
					(*m_Ad_powers[i]) = (*m_Ad_powers[i-1])*m_Ad;
				}
			}

			/**
			 *   E:=[pow(Ad,1)
					 pow(Ad,2)
					:
					pow(Ad,k)]*x0
			*/
			void updateE()
			{
				for(int i=0;i<K;i++)
				{
					dlib::set_rowm(m_E,dlib::range(i*N,i*N+N-1)) = (*m_Ad_powers[i+1]) * m_x0;
				}
			}
			/**
			 *	 F:=[ B, 0, 0,... 0
					AB, B, 0,... 0;
					AAB,AB, B,...,0;
					:
					pow(A,k-1)B,pow(A,k-2)B,pow(A,k-3),B,...,B],
				so that X=E+F*U
			*/
			void updateF()
			{
				for(int i=0;i<K;i++)
				{
					auto Ad_pow_i_Bd = (*m_Ad_powers[i])*m_Bd;//block to be repeated through (sub)-diagonal
					for(int j=0;j+i<K;j++)
					{
				
						auto rows = dlib::range((i+j)*N,(i+j)*N+N-1);
						auto cols = dlib::range(j*R,j*R+R-1);
						dlib::set_subm(m_F,rows,cols) = Ad_pow_i_Bd;
					}
				}
			}
			/// Hx:=diag(wx)
			void updateHx()
			{
				for(int i=0;i<K;i++)
				{
					for(int j=0;j<N;j++)
					{
						m_Hx(i*N+j,i*N+j) = m_wx(j);
					}
				}
				//endpoint
				for(int j=0;j<N;j++)
				{
					m_Hx((K-1)*N+j,(K-1)*N+j) = m_wx_end(j);
				}

			}
			/// Hu:=diag(wu)
			void updateHu()
			{
				for(int i=0;i<K;i++)
				{
					for(int j=0;j<R;j++)
					{
						m_Hu(i*R+j,i*R+j) = m_wu(j);
					}
				}
				//endpoint
				for(int j=0;j<R;j++)
				{
					m_Hu((K-1)*R+j,(K-1)*R+j) = m_wu(j);
				}
			}
			/// Heps:=diag(weps);
			void updateHeps()
			{
				for(int i=0;i<N;i++)
				{
					m_Heps(i,i) = m_weps(i);
				}
			}
			/** 
			 *	g1:= F^T Hx E - F^T Hx y
			*/
			void updateg1()
			{
				//copy m_y to m_y_flat
				for(int i=0;i<N;i++)
				{
					for(int j=0;j<K;j++)
					{
						m_y_flat(j*N+i) = m_y(i,j);
					}
				}
				//copy m_uset to m_uset_flat
				for(int i=0;i<R;i++)
				{
					for(int j=0;j<K;j++)
					{
						m_uset_flat(j*R+i) = m_uset(i,j);
					}
				}
				//set first part of g
				m_g1 = dlib::trans(m_F)*m_Hx*m_E
						-dlib::trans(m_F)*m_Hx*m_y_flat 
						-m_Hu*m_uset_flat;
			}
			/**
			 *	H11 := Hu + F^T Hx F
			*/
			void updateH11()
			{
				m_H11 = m_Hu + dlib::trans(m_F)*m_Hx*m_F;
			}	
			/**
			 *	A1 := [F  [-I;-I;-I;...-I]  0]	--> x upper bound soft constraint
			*  (set F part here)
			*/
			void updateA1()
			{
				set_colm(m_A1,dlib::range(0,R*K-1)) = m_F;
			}
			/**
			 *	A2 := [F  [I;I;I;...I]  0]	--> x lower bound soft constraint
			* (set F part here)
			*/
			void updateA2()
			{
				set_colm(m_A2,dlib::range(0,R*K-1)) = m_F;
			}
		private://methods for qpOASES interface matrix setup
			inline int idx(int row,int col,int nrows,int ncols) const
			{
				return row*ncols+col;
				//return col*nrows+row;
			}
			void updateqpH()
			{
				for(int i=0;i<R*K;i++)
				{
					for(int j=0;j<R*K;j++)
					{
						m_qpH[idx(i,j,nV,nV)] = m_H11(i,j);
					}
				}
				for(int i=0;i<N;i++)
				{
					for(int j=0;j<N;j++)
					{
						m_qpH[idx(i+R*K,j+R*K,nV,nV)] = m_Heps(i,j);
					}
				}
			}
			void updateqpg()
			{
				for(int i=0;i<R*K;i++)
				{
					m_qpg[i] = m_g1(i);
				}
				for(int i=0;i<N;i++)
				{
					m_qpg[i+R*K] = m_geps(i);
				}
			}
			void updateqpA()
			{
				for(int i=0;i<N*K;i++)
				{
					for(int j=0;j<nV;j++)
					{
						m_qpA[idx(i,j,nC,nV)] = m_A1(i,j);
					}
				}
				for(int i=0;i<N*K;i++)
				{
					for(int j=0;j<nV;j++)
					{
						m_qpA[idx(i+N*K,j,nC,nV)] = -m_A2(i,j);
					}
				}
			}
			void updateqplb()
			{
				for(int i=0;i<R;i++)
				{
					for(int j=0;j<K;j++)
					{
						m_qplb[j*R+i] = m_lbu_hard(i,j);
					}
				}
				//lower bound for eps is 0
			}
			void updateqpub()
			{
				for(int i=0;i<R;i++)
				{
					for(int j=0;j<K;j++)
					{
						m_qpub[j*R+i] = m_ubu_hard(i,j);
					}
				}
				for(int i=0;i<N;i++)
				{
					m_qpub[R*K+i] = m_ubeps(i);
				}
			}
			void updateqpubA()
			{
				int row;
				// state constraints
				for(int i=0;i<N;i++)
				{
					for(int j=0;j<K;j++)
					{
						row = j*N+i;
						//upper bound	F*u - 1*epsx <= ubx - E
						m_qpubA[row] = m_ubx(i,j) - m_E(row);
						//lower bound  -F*u - 1*epsx <=-lbx + E
						m_qpubA[row+N*K] = -m_lbx(i,j) + m_E(row);
					}
				}
			}
		private://methods for execution of optimization
			/**
			 *	compute_init - initial call to the qp solver: all information for description of the qp has to be provided
			*/
			void compute_init()
			{
				//prepare intermediate matrices
				updateAd();
				updateE();
				updateF();
				updateHx();
				updateHu();
				updateHeps();
				updateg1();
				updateH11();
				updateA1();
				updateA2();

				//copy intermediate matrices into arrays//@TODO: find out, whether dlib matrix data arrays can be used directly
				updateqpH();
				updateqpg();
				updateqpA();
				updateqplb();
				updateqpub();
				updateqpubA();

				//call init method
				m_qpnWSR_out = m_qpnWSR_in;
				m_qpcputime_out = m_qpcputime_in;
				m_qproblem->init(m_qpH,m_qpg,m_qpA,m_qplb,m_qpub,0,m_qpubA,m_qpnWSR_out,&m_qpcputime_out);
			}
			/**
			 * compute_hotstart - hotstart call to the qp solver: can only be used, if the quadratic term and the system dynamics have *not* changed.
			 * lower and upper bound values, linear cost terms, initial state may be modified before re-computing with hotstart.
			 * a reduced set of variables has to be provided to the qp solver, under the assumption that the other variables have not changed.
			 */
			void compute_hotstart()
			{
				//prepare intemediate matrices
				updateE();
				updateg1();

				//copy intermediate matrices into arrays
				updateqpg();
				updateqplb();
				updateqpub();
				updateqpubA();

				//call hotstart method
				m_qpnWSR_out = m_qpnWSR_in;
				m_qpcputime_out = m_qpcputime_in;
				m_qproblem->hotstart(m_qpg,m_qplb,m_qpub,0,m_qpubA,m_qpnWSR_out,&m_qpcputime_out);
			}
			/**
			 * retrieve solution from qpOASES
			 */
			void retrieveSolution()
			{
				m_qproblem->getPrimalSolution( m_qpxOpt );
				for(int i=0;i<R*K;i++)
				{
					m_U_flat(i) = m_qpxOpt[i];
				}
				set_rowm(m_X_flat,dlib::range(0,N-1)) = m_x0;
				set_rowm(m_X_flat,dlib::range(N,N*(K+1)-1)) = m_E + m_F * m_U_flat;
				for(int i=0;i<N;i++)
				{
					for(int j=0;j<=K;j++)
					{
						m_X(i,j) = m_X_flat(j*N+i);
					}
				}
				for(int i=0;i<R;i++)
				{
					for(int j=0;j<K;j++)
					{
						m_U(i,j) = m_U_flat(j*R+i);
					}
				}
				for(int i=0;i<N;i++)
				{
					m_eps(i) = m_qpxOpt[R*K+i];
				}
			}
			/**
			 * interpolate the solution using Ad_p and Bd_p
			 */
			void interpolateSolution()
			{
				dlib::matrix<double,N,1> x;
				dlib::matrix<double,R,1> u;
				for(int i=0;i<K;i++)
				{
					x = dlib::colm(m_X,i);
					u = dlib::colm(m_U,i);
					dlib::set_subm(m_resultfun.getData(),dlib::range(1,N),dlib::range(i*P,i*P)) = x;
					dlib::set_subm(m_resultfun.getData(),dlib::range(N+1,N+R),dlib::range(i*P,i*P)) = u;
					for(int j=1;j<P;j++)
					{
						x = m_Ad_p*x+m_Bd_p*u;
						dlib::set_subm(m_resultfun.getData(),dlib::range(1,N),dlib::range(i*P+j,i*P+j)) = x;
						dlib::set_subm(m_resultfun.getData(),dlib::range(N+1,N+R),dlib::range(i*P+j,i*P+j)) = u;
					}
				}
				x = dlib::colm(m_X,K);
				u = dlib::colm(m_U,K-1);
				dlib::set_subm(m_resultfun.getData(),dlib::range(1,N),dlib::range(K*P,K*P)) = x;
				dlib::set_subm(m_resultfun.getData(),dlib::range(N+1,N+R),dlib::range(K*P,K*P)) = u;
			}

		public:
			/**
			 * constructor
			 */
			LQ_OC_single_shooting()
				:m_resultfun(K*P+1,0.0)
			{
				initialize_memory();
				initialize_userdata_matrices();
				initialize_intermediate_matrices();
				initialize_interface_variables();
				m_system_changed = true;
				m_qpnWSR_in = 1e7;
				m_qpcputime_in = 1.0e7;
			}

			/**
			 * destructor
			 */
			virtual ~LQ_OC_single_shooting()
			{	
				delete_memory();
			}

			/**
			 * execute optimization for given constraints and reference
			 */
			void compute()
			{
				if(m_system_changed)
				{
					compute_init();
					m_system_changed = false;
					std::cout<<"reinitialized qp";
				}
				else
				{
					compute_hotstart();
					if(!isFeasible() || !isSolved())
					{
						m_system_changed = true;
					}
				}
				retrieveSolution();
				interpolateSolution();
			}
			/**
			 * determine whether qpOASES deems problem feasible
			 */
			bool isFeasible()const
			{
				return m_qproblem->isInfeasible()==qpOASES::BT_FALSE;
			}
			/**
			 * determine whether qpOASES deems problem solved to given precision
			 */
			bool isSolved()const
			{
				return m_qproblem->isSolved()==qpOASES::BT_TRUE;
			}
			/**
			 * change the system
			 */
			void updateSystem(const t_Ad& Ad,const t_Bd& Bd,const t_wx& wx,const t_wu& wu)
			{
				m_Ad = Ad; m_Bd = Bd; m_wx = wx; m_wu = wu;
				setSystemChanged(true);
			}
			/**
			 * change the system
			 */
			void updateSystem(const t_Ad& Ad,const t_Bd& Bd,const t_wx& wx,const t_wu& wu,const t_weps& weps)
			{
				m_Ad = Ad; m_Bd = Bd; m_wx = wx; m_wu = wu; m_weps = weps;
				setSystemChanged(true);
			}
			/**
			 * change the system
			 */
			void updateSystem(const t_Ad& Ad,const t_Bd& Bd,const t_wx& wx,const t_wu& wu,const t_weps& weps,const t_geps& geps)
			{
				m_Ad = Ad; m_Bd = Bd; m_wx = wx; m_wu = wu; m_weps = weps; m_geps = geps;
				setSystemChanged(true);
			}
			/**
			 * change initial value, reference and bounds
			 */
			void update(const t_x0& x0,const t_y& y,const t_lbx& lbx,const t_ubx& ubx,const t_lbu_hard& lbu_hard,const t_ubu_hard& ubu_hard,const t_ubeps& ubeps)
			{
				m_x0 = x0; m_y = y; m_lbx = lbx; m_ubx = ubx; m_lbu_hard = lbu_hard; m_ubu_hard = ubu_hard; m_ubeps = ubeps;
			}
			/**
			 * change initial value, reference and bounds
			 */
			void update(const t_x0& x0,const t_y& y,const t_lbx& lbx,const t_ubx& ubx)
			{
				m_x0 = x0; m_y = y; m_lbx = lbx; m_ubx = ubx;
			}
			/**
			 * recompute without hotstart
			 */
			void setSystemChanged(bool value){m_system_changed = value;}
			/**
			 * redefine time steps according to supplied end time Tend
			 */
			void setEndTime(double Tend)
			{
				double T[K+1];
				adore::mad::linspace(0.0,Tend,T,K+1);
				double dt_int = Tend/(K*P);
				for(int i=0;i<K;i++)
				{
					for(int j=0;j<P;j++)
					{
						m_resultfun.getData()(0,i*P+j) = T[i]+j*dt_int;
					}
				}
				m_resultfun.getData()(0,K*P) = T[K];
			}

		public://getter methods
			t_Ad& Ad(){return m_Ad;}//discrete time system matrix.
			t_Bd& Bd(){return m_Bd;}//discrete time input matrix
			t_Ad& Ad_p(){return m_Ad_p;}//discrete time system matrix for interpolation
			t_Bd& Bd_p(){return m_Bd_p;}//discrete time input matrix for interpolation
			t_resultfun& result_fun(){return m_resultfun;}//returns a function with linear interpolation of result
			//weights for cost function
			t_wx& wx(){return m_wx;}//weights for states (will be put on main diagonal of Hx)
			t_wu& wu(){return m_wu;}//weights for inputs (will be put on main diagonal of Hu)
			t_wx_end& wx_end(){return m_wx_end;}//weights for states (will be put on main diagonal of Hx)
			t_wu_end& wu_end(){return m_wu_end;}//weights for inputs (will be put on main diagonal of Hu)
			t_weps& weps(){return m_weps;}//quadratic weights for slack (will be put in Heps) 
			t_geps& geps(){return m_geps;}//linear weights for slack
			//online data: bounds
			t_lbx& lbx(){return m_lbx;}//lower bound on state trace
			t_ubx& ubx(){return m_ubx;}//upper bound on state trace
			t_lbu_hard& lbu_hard(){return m_lbu_hard;}//lower bound on input trace, hard constraint going to lb
			t_ubu_hard& ubu_hard(){return m_ubu_hard;}//upper bound on input trace, hard constraint going to ub
			t_ubeps& ubeps(){return m_ubeps;}//lbeps==0, softconstraints for x and/or u if ubeps>0
			//online data: x0, y
			t_x0& x0(){return m_x0;}//initial state
			t_y& y(){return m_y;}//desired states (reference) y=[y_0(t_1);y_1(t_1);y_2(t_1);y_0(t_2);y_1(t_2);y_2(t_2);...;y_0(t_K);y_1(t_K);y_2(t_K)] (for N=3)
			t_uset& uset(){return m_uset;}//set points (reference for controls)
			//output
			t_X& X(){return m_X;}
			t_U& U(){return m_U;}
			t_eps& eps(){return m_eps;}
			real_t getCPUTime(){return m_qpcputime_out;}
			int getnWSR(){return m_qpnWSR_out;}
			qpOASES::QProblem* getQProblem(){return m_qproblem;}
			void setMaxCPUTime(real_t value){m_qpcputime_in = value;}
			void setMaxnWSR(int value){m_qpnWSR_in=value;}
			int getnV(){return nV;}
			int getnC(){return nC;}
		};
	}
}