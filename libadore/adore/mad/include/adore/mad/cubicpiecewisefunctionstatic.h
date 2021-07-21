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
 *   Reza Dariani - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/mad/arraymatrixtools.h>

#include <dlib/matrix.h>

namespace adore
{
	namespace mad
	{
		/** 
		 *       x
		 *    x       x     x     x      x  x    x		<- inputPoints #N
		 *         x      x    x      x   
		 *    |  | |  |   | |  |  |   |  |  |    |		<- breaks #N. polynomials always between two input/break points
		 *        y    y     y    y       y    y		<- differently spaced/arbitrary number output points
		 */
		template<int N>//NumberOfBreak==NumberOfInputPoints==NumberOfPolynomials+1
		class CubicPiecewiseFunctionStatic
		{
		public:
			double breaks[N];
			double coef_1[N-1];
			double coef_2[N-1];
			double coef_3[N-1];
			double coef_4[N-1];

		private:
			inline void evaluate1(int i,int j,double *input_x,double *output_y,double *output_dy=0,double *output_ddy=0,double *output_dddy=0)
			{
				double local_x = (input_x[i]-breaks[j]);
				output_y[i]=((((coef_1[j]*local_x+coef_2[j])*local_x)+coef_3[j])*local_x)+coef_4[j];		
				if(output_dy)
				{
					output_dy[i]=(((3*coef_1[j]*local_x+2*coef_2[j])*local_x)+coef_3[j]);
					if(output_ddy)
					{
						output_ddy[i]=6*coef_1[j]*local_x+2*coef_2[j];
						if(output_ddy)
						{
							output_dddy[i]=6*coef_1[j];
						}
					}
				}		
			}
		public:

			void evaluate(int outN,double *input_x,double *output_y,double *output_dy=0,double *output_ddy=0,double *output_dddy=0)
			{
				double eps=1e-20;
				double inf=1.0/eps;
				//intermediate variables
				int localIndex[outN];//the poly index for each x value
				//local search algprithm
				for ( int i=0;i<outN;i++)
				{	  		
					for (int j=0;j<N;j++)
					{		 
						if( input_x[i]+eps >= breaks[j] || j==0 )
						{
							if( j==N-1 || input_x[i]+eps < breaks[j+1] )
							{
								localIndex[i]=j;
							}
						}
					}		
				} 

				//interplation	
				for (int i=0;i<outN;i++)
				{
					evaluate1(i,localIndex[i],input_x,output_y,output_dy,output_ddy,output_dddy);
				}
			}

			/**
			 * evaluate_ordered is like evaluate, but expects input_x to be an ordered sequence of points. indexing can be greatly improved by this assumption.
			 */
			void evaluate_ordered(int outN,double *input_x,double *output_y,double *output_dy=0,double *output_ddy=0,double *output_dddy=0)
			{
				int j=0;//the break index
				for (int i=0;i<outN;i++)//the input index
				{
					while( j<N-1 && input_x[i] > breaks[j+1] ) j++;
					evaluate1(i,j,input_x,output_y,output_dy,output_ddy,output_dddy);
				}
			}

			/**
			 *  based on Schöneberg and Reinsch smoothing spline
			 * 	https://link.springer.com/content/pdf/10.1007/BF02162161.pdf
			 *  https://en.wikipedia.org/wiki/Smoothing_spline
			*	https://wiki.dlr.de/confluence/display/fau/How+to+use+Fit+function
			 *  size of input_x and input_y has to be N+1
			 */
			void fit(double *input_x,double *input_y,double *input_w, double smoothingFactor)
			{
				//step by step info  : /adore/mad/src/cubic_piecewise_function.cpp
				//intermediate variables
				double dx[N-1];
				double oneDivDx[N-1];
				double dy[N-1];
				double DyDx[N-1];				
				ArrayMatrixTools::diff(dx,input_x,N);
				ArrayMatrixTools::diff(dy,input_y,N);
				for(int i=0;i<N-1;i++){oneDivDx[i]=1.0/dx[i];}
				ArrayMatrixTools::pointwise_multiply(DyDx,oneDivDx,dy,N-1);//dy/dx
				
				double dx1n_2[N-2]; //dx from 1 --> N-2
				double dx0n_3[N-2];  //dx from 0 --> N-3
				double dx2x1n_20n_3[N-2]; //2*(dx1n_2+dx0n_3)
				ArrayMatrixTools::pieceOfArray(dx1n_2,dx,1,N-2);
				ArrayMatrixTools::pieceOfArray(dx0n_3,dx,0,N-3);
				for(int i=0;i<N-2;i++){	dx2x1n_20n_3[i]=2*(dx1n_2[i]+dx0n_3[i]);}
				
				//sparse matrix intermediate variables
				double sp_dx1n_2 [(N-2)*(N-2)];
				double sp_dx0n_3 [(N-2)*(N-2)];
				double sp_dx2x1n_20n_3 [(N-2)*(N-2)]; 
				ArrayMatrixTools::sparseDiagonalMatrix(sp_dx1n_2,dx1n_2,N-2,N-2,N-2,-1);
				ArrayMatrixTools::sparseDiagonalMatrix(sp_dx0n_3,dx0n_3,N-2,N-2,N-2,1);
				ArrayMatrixTools::sparseDiagonalMatrix(sp_dx2x1n_20n_3,dx2x1n_20n_3,N-2,N-2,N-2,0);

				double R[(N-2)*(N-2)];
				for(int i=0; i<N-2;i++)  //data are from 1-->n-2 or 0-->n-3 this is why there is an extrea -2 in i<n-2-2
				{
					for(int j=0; j<N-2;j++)
					{
						R[i*(N-2)+j]=sp_dx1n_2[i*(N-2)+j]+sp_dx0n_3[i*(N-2)+j]+sp_dx2x1n_20n_3[i*(N-2)+j];
					}
				}						
				//intermediate variables
				double oneDivDx1n_2[N-2];
				double oneDivDx0n_3[N-2];
				double oneDivDx_1n_20n_3[N-2];
				
				ArrayMatrixTools::pieceOfArray(oneDivDx1n_2,oneDivDx,1,N-2);
				ArrayMatrixTools::pieceOfArray(oneDivDx0n_3,oneDivDx,0,N-3);
				for(int i=0;i<N-2;i++){
					oneDivDx_1n_20n_3[i]=-1*(oneDivDx1n_2[i]+oneDivDx0n_3[i]);}
				
				//sparse matrix intermediate variables
				double sp_oneDivDx1n_2[(N-2)*(N)];
				double sp_oneDivDx0n_3[(N-2)*(N)];
				double sp_oneDivDx_1n_20n_3[(N-2)*(N)];
				ArrayMatrixTools::sparseDiagonalMatrix(sp_oneDivDx1n_2,oneDivDx1n_2,N-2,N,N-2,2);
				ArrayMatrixTools::sparseDiagonalMatrix(sp_oneDivDx0n_3,oneDivDx0n_3,N-2,N,N-2,0);
				ArrayMatrixTools::sparseDiagonalMatrix(sp_oneDivDx_1n_20n_3,oneDivDx_1n_20n_3,N-2,N,N-2,1);

				//Qt and its transpose (see wiki link)
				double Qt[(N-2)*(N)];
				double QtT[(N)*(N-2)];

				for(int i=0; i<N-2;i++)  //data are from 1-->n-2 or 0-->n-3 this is why there is an extrea -2 in i<n-2-2
				{
					for(int j=0; j<N;j++)
					{
						Qt[i*(N)+j]=sp_oneDivDx1n_2[i*(N)+j]+sp_oneDivDx0n_3[i*(N)+j]+sp_oneDivDx_1n_20n_3[i*(N)+j];
					}
				}
				
				//intermediate variables
				double D[N];
				double sp_D[N*N];
				double QtD[(N-2)*N];
				double QtDQ[(N-2)*(N-2)];
				//double temp_M[(N-2)*(N-2)];
				for(int i=0;i<N;i++) { D[i]=1.0/input_w[i];}

				ArrayMatrixTools::sparseDiagonalMatrix(sp_D,D,N,N,N,0);		
				ArrayMatrixTools::transpose(QtT,Qt,N-2,N);
				ArrayMatrixTools::matrixMultiplication(QtD,Qt,N-2,N,sp_D,N,N);
				ArrayMatrixTools::matrixMultiplication(QtDQ,QtD,N-2,N,QtT,N,N-2);
				
				//intermediate matrix definition (dlib will be used to calculated matrix inverse)
				dlib::matrix<double> M(N-2,N-2);
				dlib::matrix<double> inv_M;
				for(int i=0; i<(N-2);i++){
					for(int j=0; j<(N-2); j++){
						M(i,j)=6*(1-smoothingFactor)*QtDQ[i*(N-2)+j]+smoothingFactor*R[i*(N-2)+j];
					}
				}
				inv_M=dlib::inv(M);
				
				//intermediate variables (dlib matrix ---> array )
				double temp_invM[(N-2)*(N-2)];
				
				for(int i=0; i<(N-2);i++)
				{
					for(int j=0; j<(N-2); j++)
					{	
						temp_invM[i*(N-2)+j]=inv_M(i,j);			
					}
				}
				//intermediate variables 
				double diff_DyDx[N-2];    
				double u[N-2];
				double u_n[N]; //in order to add zero at beginning and at the end
				double diff_u_n[N-1];

				ArrayMatrixTools::diff(diff_DyDx,DyDx,N-1);
				ArrayMatrixTools::matrixMultiplication(u,temp_invM,N-2,N-2,diff_DyDx,N-2,1);
				u_n[0]=0;  //at beggining
				u_n[N-1]=0; //at end
				for(int i=0; i<N-2; i++)  {u_n[i+1]=u[i];}
				ArrayMatrixTools::diff(diff_u_n,u_n,N); //   matlab equivalent --> diff([zeros(1,yd); u; zeros(1,yd)])
				
				 //intermediate variables
				double diff_u_n_oneDivDx [N-1];
				double diff_unoneDivDx_n1[N+1]; //to add a zero at beggining and the end (needed for diff)
				double diff2_unoneDivDx_n1[N];
				ArrayMatrixTools::pointwise_multiply(diff_u_n_oneDivDx,diff_u_n,oneDivDx,N-1); // matlab equivalent -->diff([zeros(1,yd); u; zeros(1,yd)])./dx(:,dd)
				diff_unoneDivDx_n1[0]=0; //at beginning
				diff_unoneDivDx_n1[N]=0; //at end
				for(int i=0; i<N-1; i++) {diff_unoneDivDx_n1[i+1]=diff_u_n_oneDivDx[i];}
				ArrayMatrixTools::diff(diff2_unoneDivDx_n1,diff_unoneDivDx_n1,N+1);

				//Finaly calculating smooth y
				double yi[N];
				for(int i=0; i<N; i++)		
				{
					yi[i]=input_y[i]-(6*(1-smoothingFactor))*sp_D[i*N+i]*diff2_unoneDivDx_n1[i];
				}

				//Converting y to cubic coeeficients which results is y = coef1*h^3 + coef2*h^2 + coef3*h + coef4

				//intermediate varialbles
				double c3[N];
				double diff_yi[N-1];
				double diff_c3[N-1];
				double c2[N-1];				
				
				c3[0]=0;
				c3[N-1]=0;
				for(int i=0; i<N-2; i++)  {c3[i+1]=u[i]*smoothingFactor;}
				ArrayMatrixTools::diff(diff_yi,yi,N);
				for(int i=0; i<N-1; i++) {c2[i]=diff_yi[i]*oneDivDx[i]-dx[i]*(2*c3[i]+c3[i+1]);}
				ArrayMatrixTools::diff(diff_c3,c3,N);
				for(int i=0; i<N-1; i++)
				{
					breaks[i] = input_x[i];
					coef_1[i]=diff_c3[i]*oneDivDx[i];
					coef_2[i]=3*c3[i];
					coef_3[i] = c2[i];
					coef_4[i] = yi[i];
				}
				breaks[N-1] = input_x [N-1]; 

			}


			
		};
	}
}