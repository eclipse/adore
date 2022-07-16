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
#include <adore/mad/cubicpiecewisefunction.h>
using namespace adore::mad;
const double eps = 1e-20;
const double inf = 1.0 / eps;
//************************************************************
//This function read the breaks and coefs of piecewise polynomial 
//and builds a structure
void CubicPiecewiseFunction::toPolynomialFrom(PieceweisePolynomial* pp, double *input_breaks, double *input_coef1, double *input_coef2, double *input_coef3, double *input_coef4, int inputLength)
{
	pp->breaks = new double[inputLength + 1];
	pp->coef_1 = new double[inputLength];
	pp->coef_2 = new double[inputLength];
	pp->coef_3 = new double[inputLength];
	pp->coef_4 = new double[inputLength];
	pp->dimension = inputLength;
	for (int i = 0; i < inputLength; i++)
	{
		pp->breaks[i] = input_breaks[i];
		pp->coef_1[i] = input_coef1[i];
		pp->coef_2[i] = input_coef2[i];
		pp->coef_3[i] = input_coef3[i];
		pp->coef_4[i] = input_coef4[i];
	}
	pp->breaks[inputLength] = input_breaks[inputLength];
}
//*******************************************************
void CubicPiecewiseFunction::BreaksGenerator(double *Userbreaks, int UserbreaksLength, double start, double resolution)
{
	Userbreaks[0] = start; //coordination
	for (int i = 1; i < UserbreaksLength; i++)
	{
		Userbreaks[i] = Userbreaks[i - 1] + resolution;
	}
}
std::vector<double> CubicPiecewiseFunction::BreaksGenerator(double start, double end, int NumberOfPoints)
{
	std::vector<double> output;
	double resolution = (end - start) / (NumberOfPoints-1);
	output.push_back(start);
	for (int i = 1; i < NumberOfPoints; i++)
	{
		output.push_back(output.at(i-1) + resolution);
	}
	output.back() = end;
	return output;
}
//*******************************************************
void CubicPiecewiseFunction::BreaksGeneratorExp(double *Userbreaks, int NumOfPoints, double start, double end)
{
	double EPS = 1e-1;
	double y0 = 0; 
	double y1 = std::log10(end-start);
	double resolution = (y1-y0) / NumOfPoints;
	double *y;
	y = new double [NumOfPoints];
	BreaksGenerator(&y[0],NumOfPoints,y0,resolution);
	for (int i = 0; i < NumOfPoints; i++)
	{
		Userbreaks[i] = (std::pow(10.0,y[i])-1) + start;
	}
	Userbreaks[NumOfPoints-1] = end - EPS;
	delete [] y;
}
//*********************************************
CubicPiecewiseFunction::LocalCoordination CubicPiecewiseFunction::localCoordination(double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
	LocalCoordination lc;
	lc.lc_breaks.resize(UserbreaksLength);
	lc.lc_index.resize(UserbreaksLength);
	for (int i = 0; i < UserbreaksLength; i++)
	{
		for (int j = 0; j <= pp.dimension - 1; j++)
		{
			if (Userbreaks[i] + eps >= pp.breaks[j] && Userbreaks[i] + eps <= pp.breaks[j + 1]) 
			{
				lc.lc_index[i] = j;
			}
		}
	}
	for (int i = 0; i < UserbreaksLength; i++) 
	{		//Local coordination of high resoluted data		

		lc.lc_breaks[i] = (Userbreaks[i] - pp.breaks[lc.lc_index[i]]);
	}	
	return lc;
}
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
	auto lc = localCoordination(Userbreaks,UserbreaksLength,pp);
	//ax^3+bx^2+cx+d
	for (int i = 0; i < UserbreaksLength; i++)
	{		
		interpolatedSpline[i] = ((((pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + pp.coef_2[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_3[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_4[lc.lc_index[i]];			
	}
}

//***********************************************
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *d_interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
	auto lc = localCoordination(Userbreaks,UserbreaksLength,pp);
	for (int i = 0; i < UserbreaksLength; i++)
	{
		interpolatedSpline[i] = ((((pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + pp.coef_2[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_3[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_4[lc.lc_index[i]];
		d_interpolatedSpline[i] = (((3 * pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + 2 * pp.coef_2[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_3[lc.lc_index[i]]);//*dt;
	}
}

//***********************************************
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *d_interpolatedSpline, double *dd_interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
		auto lc = localCoordination(Userbreaks,UserbreaksLength,pp);
	for (int i = 0; i < UserbreaksLength; i++)
	{
		interpolatedSpline[i] = ((((pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + pp.coef_2[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_3[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_4[lc.lc_index[i]];
		d_interpolatedSpline[i] = (((3 * pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + 2 * pp.coef_2[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_3[lc.lc_index[i]]);//*dt;
		dd_interpolatedSpline[i] = 6 * pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + 2 * pp.coef_2[lc.lc_index[i]];//*dt;
	}

}
//***********************************************
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *d_interpolatedSpline, double *dd_interpolatedSpline, double *ddd_interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
		auto lc = localCoordination(Userbreaks,UserbreaksLength,pp);
	for (int i = 0; i < UserbreaksLength; i++)
	{
		interpolatedSpline[i] = ((((pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + pp.coef_2[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_3[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_4[lc.lc_index[i]];
		d_interpolatedSpline[i] = (((3 * pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + 2 * pp.coef_2[lc.lc_index[i]])*lc.lc_breaks[i]) + pp.coef_3[lc.lc_index[i]]);//*dt;
		dd_interpolatedSpline[i] = 6 * pp.coef_1[lc.lc_index[i]] * lc.lc_breaks[i] + 2 * pp.coef_2[lc.lc_index[i]];//*dt;
		ddd_interpolatedSpline[i] = 6 * pp.coef_1[lc.lc_index[i]];
	}
}
//***********************************************
void CubicPiecewiseFunction::repeatedValueInterpolation(double *output_x, double *output_y, double *output_weight, int *outputLength, double *input_x, double *input_y, double *input_weight, int inputLength)
{
	//check for repeated value of x axis, if there is any, an interpolation by considering their importance (weight) is done
	double *diff_X;
	int diff_X_length = inputLength - 1;
	diff_X = new double[diff_X_length];
	int *x_ind;
	x_ind = new int[inputLength];
	double *temp_x, *temp_y, *temp_w;
	temp_x = new double[inputLength];
	temp_y = new double[inputLength];
	temp_w = new double[inputLength];
	//1 : check if data on the x (axis) are ascending
	ArrayMatrixTools::diff(diff_X, input_x, inputLength);   	 
	if (ArrayMatrixTools::any(diff_X, diff_X_length, "L", 0.0))  //sorting is needed
	{
		printf("There is a repetetive value, sorting is necessary\n");
		ArrayMatrixTools::sort(temp_x, x_ind, input_x, inputLength);
		for (int i = 0; i < inputLength; i++)
		{
			temp_y[i] = input_y[x_ind[i]];
			temp_w[i] = input_weight[x_ind[i]];
		} 	
		std::copy(temp_x, temp_x + inputLength, input_x);
		std::copy(temp_y, temp_y + inputLength, input_y);
		std::copy(temp_w, temp_w + inputLength, input_weight);
	} //end if
	delete[] temp_x;
	delete[] temp_y;
	delete[] temp_w;
	//2: check for repetitive value, which means diff of data is equal to zero
	ArrayMatrixTools::diff(diff_X, input_x, inputLength);
	if (ArrayMatrixTools::any(diff_X, diff_X_length, "E", 0.0) == false)  //Data are perfect, no further calculation is needed, no repetition on x
	{
		std::copy(input_x, input_x + inputLength, output_x);
		std::copy(input_y, input_y + inputLength, output_y);
		std::copy(input_weight, input_weight + inputLength, output_weight);
		*outputLength = inputLength;
		delete[] x_ind;
		delete[] diff_X;
		return;
	}
	printf("\nThere is repetetive value");
	//********************************************************************************
	// from here till end is about the case of repetetive value on x, and necssity of deleting them and replacing them by a good interpolation by considering their importance (weight)
	//********************************************************************************
	int *repeatedValueIndex;
	repeatedValueIndex = new int[inputLength];
	int repeatedValueIndexLength = 0;
	ArrayMatrixTools::find(repeatedValueIndex, &repeatedValueIndexLength, diff_X, diff_X_length, "G", 0.0);
	int *index_tmp1, *index_tmp2, *repetitionVector;
	index_tmp1 = new int[repeatedValueIndexLength + 1];
	index_tmp2 = new int[repeatedValueIndexLength + 1];
	repetitionVector = new int[repeatedValueIndexLength + 1];
	index_tmp1[0] = 0;
	for (int i = 0; i < repeatedValueIndexLength; i++)
	{
		index_tmp1[i + 1] = repeatedValueIndex[i] + 1;
	}

	std::copy(index_tmp1 + 1, index_tmp1 + repeatedValueIndexLength + 1, index_tmp2);
	index_tmp2[repeatedValueIndexLength] = inputLength;

	for (int i = 0; i < repeatedValueIndexLength + 1; i++)
	{
		repetitionVector[i] = index_tmp2[i] - index_tmp1[i];
	}
	double *new_y_interpolated;
	new_y_interpolated = new double[repeatedValueIndexLength + 1];
	dlib::matrix<double> sumw, wvec, y_array;
	sumw = dlib::ones_matrix<double>(repeatedValueIndexLength + 1, 1) - 1;  //sum of weight of repeated value
	wvec = dlib::ones_matrix<double>(repeatedValueIndexLength + 1, 1) - 1;
	y_array = dlib::ones_matrix<double>(1, repeatedValueIndexLength + 1) - 1;
	for (int i = 0; i < repeatedValueIndexLength + 1; i++)
	{
		new_y_interpolated[i] = input_y[index_tmp1[i]];
		sumw(i, 0) = input_weight[index_tmp1[i]];
	}
	int *ind;
	ind = new int[repeatedValueIndexLength + 1];
	for (int i = 0; i < repeatedValueIndexLength + 1; i++)
	{
		if (repetitionVector[i] > 1)     //repeated value
		{
			sumw(i, 0) = 0;
			y_array = dlib::ones_matrix<double>(1, repeatedValueIndexLength + 1) - 1;
			for (int j = 1; j <= repetitionVector[i]; j++)
			{
				ind[j] = j + (index_tmp1[i] - 1);
				wvec(j, 0) = input_weight[ind[j]];
				sumw(i, 0) = sumw(i, 0) + wvec(j, 0);
				y_array(0, j) = input_y[ind[j]];
			}
			new_y_interpolated[i] = (y_array*wvec) / sumw(i, 0);
		}
		output_x[i] = input_x[index_tmp1[i]];
		output_y[i] = new_y_interpolated[i];
		output_weight[i] = sumw(i, 0);
	}	
	*outputLength =  repeatedValueIndexLength;
	delete[] diff_X;	
	delete[] repeatedValueIndex;
	delete[] index_tmp1;
	delete[] index_tmp2;
	delete[] repetitionVector;
	delete[] new_y_interpolated;
	delete[] ind;
}

//*******************************************************
void CubicPiecewiseFunction::fit(PieceweisePolynomial* pp, double *input_x, double *input_y, double *input_w, int inputLength, double smoothingFactor)
{
	//TODO : other regression methids must be added here
	smoothingSpline( pp,  &input_x[0], &input_y[0], &input_w[0],  inputLength,  smoothingFactor);

}
void CubicPiecewiseFunction::smoothingSpline(PieceweisePolynomial* pp, double *input_x, double *input_y, double *input_w, int inputLength, double smoothingFactor)
{
	/*
	based on Schöneberg and Reinsch smoothing spline
	short description is given here, for more info see below links
	https://link.springer.com/content/pdf/10.1007/BF02162161.pdf
	https://en.wikipedia.org/wiki/Smoothing_spline
	https://wiki.dlr.de/confluence/display/fau/How+to+use+Fit+function
	
	cubic polynomial form f(x)=a + b(x-xi)+c(x-xi)^2+d(x-xi)^3
	continuity and smoothing conditions must be fullfilled
	 1) f''(x+) = f''(x-) 
	 2) f(x+) = f (x-)
	 3) f'(x+) = f' (x-)
	 general formula consists of two parts data fitting and smoothing and smoothingFactor is the weight between parts
	*/

	double *yi, *xi, *wi;
	xi = new double[inputLength];
	yi = new double[inputLength];
	wi = new double[inputLength];
	int n;
	repeatedValueInterpolation(xi, yi, wi, &n, input_x, input_y, input_w, inputLength);
	/*Step one: natural end condition f''(x1) = f''(xN) = 0 
	any other points f''(x+) = f''(x-) 
	which results in c_i-1 .Dx_i-1 + c_i.2.(Dx_i-1 + Dx_i) + c_i+1.D_x = 3(Da_i/Dx_i - Da_i-1/Dx_i-1)
	in matrix form Rc = 3 Q_transpose a
	c := (c_i) [2 --> N-1]
	a := (a_i) [1 --> N]
	below we try to build R
	R is a symetric tridiagonal matrix of order N-2  (Dx_i-1, 2(Dx_i-1 + Dx_i), Dx_i)
	*/
	double *dx, *dy, *DyDx, *oneDivDx;  
	dx = new double[n];
	oneDivDx = new double[n];
	dy = new double[n];
	DyDx = new double[n - 1];
	ArrayMatrixTools::diff(dx, xi, n);
	ArrayMatrixTools::diff(dy, yi, n);
	for (int i = 0; i < n - 1; i++)
	{
		oneDivDx[i] = 1 / dx[i];
	}

	ArrayMatrixTools::pointwise_multiply(DyDx, oneDivDx, dy, n - 1);
	double *dx1n_2, *dx0n_3, *dx2x1n_20n_3;	
	dx1n_2 = new double[n - 2]; ///dx from 1 to N-2 (Dx_i-1)
	dx0n_3 = new double[n - 2];  //dx from 0 to N-3 (D_xi)
	dx2x1n_20n_3 = new double[n - 2]; //2(Dx_i-1 + Dx_i)
	ArrayMatrixTools::pieceOfArray(dx1n_2, dx, 1, n - 2);  //dx from 1 to N-2 
	ArrayMatrixTools::pieceOfArray(dx0n_3, dx, 0, n - 3);  
	for (int i = 0; i < n - 2; i++) {
		dx2x1n_20n_3[i] = 2 * (dx1n_2[i] + dx0n_3[i]);
	}
	double *sp_dx1n_2, *sp_dx0n_3, *sp_dx2x1n_20n_3;     //sp referes to sparse, this matrices they have the same size
	sp_dx1n_2 = new double[(n - 2)*(n - 2)];
	sp_dx0n_3 = new double[(n - 2)*(n - 2)];
	sp_dx2x1n_20n_3 = new double[(n - 2)*(n - 2)];
	ArrayMatrixTools::sparseDiagonalMatrix(sp_dx1n_2, dx1n_2, n - 2, n - 2, n - 2, -1);  //sparsing to new size
	ArrayMatrixTools::sparseDiagonalMatrix(sp_dx0n_3, dx0n_3, n - 2, n - 2, n - 2, 1);
	ArrayMatrixTools::sparseDiagonalMatrix(sp_dx2x1n_20n_3, dx2x1n_20n_3, n - 2, n - 2, n - 2, 0);
	double *R;
	R = new double[(n - 2)*(n - 2)];
	for (int i = 0; i < n - 2; i++) 
	{
		for (int j = 0; j < n - 2; j++)
		{
			R[i*(n - 2) + j] = sp_dx1n_2[i*(n - 2) + j] + sp_dx0n_3[i*(n - 2) + j] + sp_dx2x1n_20n_3[i*(n - 2) + j];
		}
	}
/*
Build Q_transpose
Q_transpose is a tridiagonal matrix of order (N-2)x(N)
general row 1/Dx_i-1  , -1/Dx_i-1 - 1*Dx_i , 1/Dx_i
Process is similar to the R
*/
	double *oneDivDx1n_2 = nullptr, *oneDivDx0n_3, *oneDivDx_1n_20n_3;
	oneDivDx1n_2 = new double[n - 2];
	oneDivDx0n_3 = new double[n - 2];
	oneDivDx_1n_20n_3 = new double[n - 2];
	ArrayMatrixTools::pieceOfArray(oneDivDx1n_2, oneDivDx, 1, n - 2);
	ArrayMatrixTools::pieceOfArray(oneDivDx0n_3, oneDivDx, 0, n - 3);
	for (int i = 0; i < n - 2; i++) {
		oneDivDx_1n_20n_3[i] = -1 * (oneDivDx1n_2[i] + oneDivDx0n_3[i]);
	}
	double *sp_oneDivDx1n_2, *sp_oneDivDx0n_3, *sp_oneDivDx_1n_20n_3;     //sp referes to sparse
	sp_oneDivDx1n_2 = new double[(n - 2)*(n)];
	sp_oneDivDx0n_3 = new double[(n - 2)*(n)];
	sp_oneDivDx_1n_20n_3 = new double[(n - 2)*(n)];
	ArrayMatrixTools::sparseDiagonalMatrix(sp_oneDivDx1n_2, oneDivDx1n_2, n - 2, n, n - 2, 2);
	ArrayMatrixTools::sparseDiagonalMatrix(sp_oneDivDx0n_3, oneDivDx0n_3, n - 2, n, n - 2, 0);
	ArrayMatrixTools::sparseDiagonalMatrix(sp_oneDivDx_1n_20n_3, oneDivDx_1n_20n_3, n - 2, n, n - 2, 1);
	double *Qt, *QtT;
	Qt = new double[(n - 2)*(n)];
	QtT = new double[(n)*(n - 2)];
	for (int i = 0; i < n - 2; i++)  //data are from 1-->n-2 or 0-->n-3 this is why there is an extrea -2 in i<n-2-2
	{
		for (int j = 0; j < n; j++)
		{
			Qt[i*(n)+j] = sp_oneDivDx1n_2[i*(n)+j] + sp_oneDivDx0n_3[i*(n)+j] + sp_oneDivDx_1n_20n_3[i*(n)+j];
		}
	}
	double *D, *sp_D, *QtD, *QtDQ, *temp_M;
	D = new double[n];
	sp_D = new double[n*n];
	QtD = new double[(n - 2)*n];
	QtDQ = new double[(n - 2)*(n - 2)];
	temp_M = new double[(n - 2)*(n - 2)];
	for (int i = 0; i < n; i++)
	{
		D[i] = 1 / wi[i];
	}
/*
equation (4) from https://link.springer.com/content/pdf/10.1007/BF02162161.pdf
can be written as Mc = 3pQ_transpose y
M = 6(1-p)Q_t D^2 Q + pR
D := variance in y
p := smoothing factor [0, 1]
a = y - 6(1-p)D^2Qu
c = 3pu

*/
	ArrayMatrixTools::sparseDiagonalMatrix(sp_D, D, n, n, n, 0);
	ArrayMatrixTools::transpose(QtT, Qt, n - 2, n);
	ArrayMatrixTools::matrixMultiplication(QtD, Qt, n - 2, n, sp_D, n, n);
	ArrayMatrixTools::matrixMultiplication(QtDQ, QtD, n - 2, n, QtT, n, n - 2);
	dlib::matrix<double> M(n - 2, n - 2);
	dlib::matrix<double> inv_M;
	for (int i = 0; i < (n - 2); i++) {
		for (int j = 0; j < (n - 2); j++) {
			M(i, j) = 6 * (1 - smoothingFactor)*QtDQ[i*(n - 2) + j] + smoothingFactor * R[i*(n - 2) + j];
		}
	}
	double *temp_invM;
	temp_invM = new double[(n - 2)*(n - 2)];
	inv_M = dlib::inv(M);
	for (int i = 0; i < (n - 2); i++) {
		for (int j = 0; j < (n - 2); j++) {
			temp_invM[i*(n - 2) + j] = inv_M(i, j);
		}
	}
	double *diff_DyDx, *u;
	diff_DyDx = new double[n - 2];
	u = new double[n - 2];
	ArrayMatrixTools::diff(diff_DyDx, DyDx, n - 1);
	ArrayMatrixTools::matrixMultiplication(u, temp_invM, n - 2, n - 2, diff_DyDx, n - 2, 1);

	double *u_n, *diff_u_n;
	u_n = new double[n]; //in order to add zero at beginning and at the end
	diff_u_n = new double[n - 1];
	u_n[0] = 0;
	u_n[n - 1] = 0;
	for (int i = 0; i < n - 2; i++)
		u_n[i + 1] = u[i];

	ArrayMatrixTools::diff(diff_u_n, u_n, n); //diff([zeros(1,yd); u; zeros(1,yd)])
	double *diff_u_n_oneDivDx;
	diff_u_n_oneDivDx = new double[n - 1];
	ArrayMatrixTools::pointwise_multiply(diff_u_n_oneDivDx, diff_u_n, oneDivDx, n - 1); //// diff([zeros(1,yd); u; zeros(1,yd)])./dx(:,dd)

	double *diff_unoneDivDx_n1; //to add a zero at beggining and the end (needed for diff)
	diff_unoneDivDx_n1 = new double[n + 1];
	diff_unoneDivDx_n1[0] = 0;
	diff_unoneDivDx_n1[n] = 0;
	for (int i = 0; i < n - 1; i++)
		diff_unoneDivDx_n1[i + 1] = diff_u_n_oneDivDx[i];

	double *diff2_unoneDivDx_n1;
	diff2_unoneDivDx_n1 = new double[n];
	ArrayMatrixTools::diff(diff2_unoneDivDx_n1, diff_unoneDivDx_n1, n + 1);

	for (int i = 0; i < n; i++)
	{
		yi[i] = yi[i] - (6 * (1 - smoothingFactor))*sp_D[i*n + i] * diff2_unoneDivDx_n1[i];
	}

	double *c3;
	c3 = new double[n];
	c3[0] = 0;
	c3[n - 1] = 0;
	for (int i = 0; i < n - 2; i++)
		c3[i + 1] = u[i] * smoothingFactor;

	double *diff_yi;
	diff_yi = new double[n - 1];
	ArrayMatrixTools::diff(diff_yi, yi, n);
	double *c2;
	c2 = new double[n - 1];
	for (int i = 0; i < n - 1; i++)
		c2[i] = diff_yi[i] * oneDivDx[i] - dx[i] * (2 * c3[i] + c3[i + 1]);

	double *coef_1, *coef_2, *diff_c3;
	coef_1 = new double[n - 1];
	coef_2 = new double[n - 1];
	diff_c3 = new double[n - 1];
	ArrayMatrixTools::diff(diff_c3, c3, n);
	for (int i = 0; i < n - 1; i++) {
		coef_1[i] = diff_c3[i] * oneDivDx[i];
		coef_2[i] = 3 * c3[i];
	}
	toPolynomialFrom(pp, xi, coef_1, coef_2, c2, yi, n - 1);
	//deleteing all dynamic array
	delete[] xi;
	delete[] yi;
	delete[] wi;
	delete[] dx1n_2;
	delete[] dx0n_3;
	delete[] dx2x1n_20n_3;
	delete[] dx;
	delete[] dy;
	delete[] DyDx;
	delete[] oneDivDx;
	delete[] sp_dx1n_2;
	delete[] sp_dx0n_3;
	delete[] sp_dx2x1n_20n_3;
	delete[] R;
	delete[] oneDivDx1n_2;
	delete[] oneDivDx0n_3;
	delete[] oneDivDx_1n_20n_3;
	delete[] sp_oneDivDx1n_2;
	delete[] sp_oneDivDx0n_3;
	delete[] sp_oneDivDx_1n_20n_3;
	delete[]  Qt;
	delete[] QtT;
	delete[]  temp_invM;
	delete[] D;
	delete[] sp_D;
	delete[] QtD;
	delete[] QtDQ;
	delete[] temp_M;
	delete[] diff_u_n_oneDivDx;
	delete[] u_n;
	delete[] diff_u_n;
	delete[] diff_DyDx;
	delete[] u;
	delete[] diff_unoneDivDx_n1;
	delete[] diff2_unoneDivDx_n1;
	delete[] c3;
	delete[] diff_yi;
	delete[] c2;
	delete[] coef_1;
	delete[] coef_2;
	delete[] diff_c3;
}
void CubicPiecewiseFunction::deleteCubicSplinepp(PieceweisePolynomial &pp)
{
	delete[] pp.breaks;
	delete[] pp.coef_1;
	delete[] pp.coef_2;
	delete[] pp.coef_3;
	delete[] pp.coef_4;
}

void CubicPiecewiseFunction::deleteCubicSplinepp1(PieceweisePolynomial* pp)
{
	delete[] pp->breaks;
	delete[] pp->coef_1;
	delete[] pp->coef_2;
	delete[] pp->coef_3;
	delete[] pp->coef_4;
}

//----------------------------------------------------------------------
int CubicPiecewiseFunction::findIndex(double point, PieceweisePolynomial &pp)
{

	int index = -1;
	for(int i = 0; i < pp.dimension; i++)
	{
		if(point >= pp.breaks[i]  && point <= pp.breaks[i+1] )
		{
			index = i;
			break;
		}
	}
	if(index == -1)
	{
		if(point < pp.breaks[0])
		{
			index = 0;
		}
		if(point > pp.breaks[pp.dimension])
		{
			index = pp.dimension;
		}
	}
	if(index==-1) printf("There is a problem to find index (adore_MAD_CPF_findIndex), %f\n",point );
	return index;
}

//--------------------------
double CubicPiecewiseFunction::splineEvaluation(int index, double point, PieceweisePolynomial &pp )
{
	double s1 = point-pp.breaks[index];
	double s2 = s1 * s1;
	double s3 = s2 * s1;
	return pp.coef_1[index]*s3 + pp.coef_2[index]*s2 + pp.coef_3[index]*s1 + pp.coef_4[index];
}