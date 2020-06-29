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
 *   Reza Deriani - initial API and implementation
 ********************************************************************************/

#include <adore/mad/cubicpiecewisefunction.h>
using namespace adore::mad;
const double eps = 1e-20;
const double inf = 1.0 / eps;






//************************************************************
//This function read the breaks and coefs of piecewise polynomial 
//and builds a structure
void CubicPiecewiseFunction::mkpp(PieceweisePolynomial* pp, double *input_breaks, double *input_coef1, double *input_coef2, double *input_coef3, double *input_coef4, int inputLength)
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
	double resolution = (end - start) / NumberOfPoints;
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
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
	int *localIndex;
	localIndex = new int[UserbreaksLength];
	for (int i = 0; i < UserbreaksLength; i++)
	{

		for (int j = 0; j <= pp.dimension - 1; j++)
		{
			if (Userbreaks[i] + eps >= pp.breaks[j] && Userbreaks[i] + eps <= pp.breaks[j + 1]) {
				localIndex[i] = j;
			}
		}
	}

	double *localUserBreaks;
	localUserBreaks = new double[UserbreaksLength];
	for (int i = 0; i < UserbreaksLength; i++) {		//Local coordination of high resoluted data		

		localUserBreaks[i] = (Userbreaks[i] - pp.breaks[localIndex[i]]);
	}

	//ax^3+bx^2+cx+d

	for (int i = 0; i < UserbreaksLength; i++)
	{
		
		interpolatedSpline[i] = ((((pp.coef_1[localIndex[i]] * localUserBreaks[i] + pp.coef_2[localIndex[i]])*localUserBreaks[i]) + pp.coef_3[localIndex[i]])*localUserBreaks[i]) + pp.coef_4[localIndex[i]];
		//cout<<"\n"<<interpolatedSpline[i];			
	}

	delete[] localIndex;
	delete[] localUserBreaks;
}

//***********************************************
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *d_interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{

	int *localIndex;
	localIndex = new int[UserbreaksLength];
	int lastJ = 0;
	for (int i = 0; i < UserbreaksLength; i++)
	{
		//localIndex[i] = 0;
		for (int j = lastJ; j <= pp.dimension - 1; j++)
		{
			if (Userbreaks[i]  >= pp.breaks[j] && Userbreaks[i]  <= pp.breaks[j + 1]) {
				localIndex[i] = j;
				lastJ = j;
				break;
			}
		}
	}
	double *localUserBreaks;
	localUserBreaks = new double[UserbreaksLength];
	for (int i = 0; i < UserbreaksLength; i++) {		//Local coordination of high resoluted data		
		//std::cout<<"\n"<<i<<"\t"<<Userbreaks[i]<<"\t"<<pp.breaks[localIndex[i]];
		localUserBreaks[i] = (Userbreaks[i] - pp.breaks[localIndex[i]]);
		//printf("localindex: i:%i \t li: %i\n",i,localIndex[i]);
	}
	//ax^3+bx^2+cx+d
	// dt unused?
	// double dt = Userbreaks[1] - Userbreaks[0];
	for (int i = 0; i < UserbreaksLength; i++)
	{
		interpolatedSpline[i] = ((((pp.coef_1[localIndex[i]] * localUserBreaks[i] + pp.coef_2[localIndex[i]])*localUserBreaks[i]) + pp.coef_3[localIndex[i]])*localUserBreaks[i]) + pp.coef_4[localIndex[i]];
		d_interpolatedSpline[i] = (((3 * pp.coef_1[localIndex[i]] * localUserBreaks[i] + 2 * pp.coef_2[localIndex[i]])*localUserBreaks[i]) + pp.coef_3[localIndex[i]]);//*dt;
	}
	delete[] localIndex;
	delete[] localUserBreaks;
}

//***********************************************
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *d_interpolatedSpline, double *dd_interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
	int *localIndex;
	localIndex = new int[UserbreaksLength];
	for (int i = 0; i < UserbreaksLength; i++)
	{
		for (int j = 0; j <= pp.dimension - 1; j++)
		{
			if (Userbreaks[i] + eps >= pp.breaks[j] && Userbreaks[i] + eps < pp.breaks[j + 1]) {
				localIndex[i] = j;
			}
		}
	}
	double *localUserBreaks;
	localUserBreaks = new double[UserbreaksLength];
	for (int i = 0; i < UserbreaksLength; i++) {		//Local coordination of high resoluted data		
		localUserBreaks[i] = (Userbreaks[i] - pp.breaks[localIndex[i]]);
	}
	//ax^3+bx^2+cx+d
	// dt unused?
	// double dt = Userbreaks[1] - Userbreaks[0];
	for (int i = 0; i < UserbreaksLength; i++)
	{
		interpolatedSpline[i] = ((((pp.coef_1[localIndex[i]] * localUserBreaks[i] + pp.coef_2[localIndex[i]])*localUserBreaks[i]) + pp.coef_3[localIndex[i]])*localUserBreaks[i]) + pp.coef_4[localIndex[i]];
		d_interpolatedSpline[i] = (((3 * pp.coef_1[localIndex[i]] * localUserBreaks[i] + 2 * pp.coef_2[localIndex[i]])*localUserBreaks[i]) + pp.coef_3[localIndex[i]]);//*dt;
		dd_interpolatedSpline[i] = 6 * pp.coef_1[localIndex[i]] * localUserBreaks[i] + 2 * pp.coef_2[localIndex[i]];//*dt;
	}
	delete[] localIndex;
	delete[] localUserBreaks;
}
//***********************************************
void CubicPiecewiseFunction::CubicSplineEvaluation(double *interpolatedSpline, double *d_interpolatedSpline, double *dd_interpolatedSpline, double *ddd_interpolatedSpline, double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp)
{
	int *localIndex;
	localIndex = new int[UserbreaksLength];
	for (int i = 0; i < UserbreaksLength; i++)
	{
		for (int j = 0; j <= pp.dimension - 1; j++)
		{
			if (Userbreaks[i] + eps >= pp.breaks[j] && Userbreaks[i] + eps < pp.breaks[j + 1]) {
				localIndex[i] = j;
			}
		}
	}
	double *localUserBreaks;
	localUserBreaks = new double[UserbreaksLength];
	for (int i = 0; i < UserbreaksLength; i++) {		//Local coordination of high resoluted data		
		localUserBreaks[i] = (Userbreaks[i] - pp.breaks[localIndex[i]]);
	}
	//ax^3+bx^2+cx+d
	// dt unused?
	// double dt = Userbreaks[1] - Userbreaks[0];
	for (int i = 0; i < UserbreaksLength; i++)
	{
		interpolatedSpline[i] = ((((pp.coef_1[localIndex[i]] * localUserBreaks[i] + pp.coef_2[localIndex[i]])*localUserBreaks[i]) + pp.coef_3[localIndex[i]])*localUserBreaks[i]) + pp.coef_4[localIndex[i]];
		d_interpolatedSpline[i] = (((3 * pp.coef_1[localIndex[i]] * localUserBreaks[i] + 2 * pp.coef_2[localIndex[i]])*localUserBreaks[i]) + pp.coef_3[localIndex[i]]);//*dt;
		dd_interpolatedSpline[i] = 6 * pp.coef_1[localIndex[i]] * localUserBreaks[i] + 2 * pp.coef_2[localIndex[i]];//*dt;
		ddd_interpolatedSpline[i] = 6 * pp.coef_1[localIndex[i]];
	}
	delete[] localIndex;
	delete[] localUserBreaks;
}
//***********************************************
void CubicPiecewiseFunction::PreProcess(double *output_x, double *output_y, double *output_weight, int *outputLength, double *input_x, double *input_y, double *input_weight, int inputLength)
{
	double *diff_X;
	int diff_X_length = inputLength - 1;
	diff_X = new double[diff_X_length];
	int *x_ind;
	x_ind = new int[inputLength];
	double *temp_x, *temp_y, *temp_w;
	temp_x = new double[inputLength];
	temp_y = new double[inputLength];
	temp_w = new double[inputLength];
	ArrayMatrixTools::diff(diff_X, input_x, inputLength);   //to check data is ascending	 
	if (ArrayMatrixTools::any(diff_X, diff_X_length, "L", 0.0))  //Data are not ascending, sorting is necessary
	{
		printf("Data are not ascending, sorting is necessary\n");
		ArrayMatrixTools::sort(temp_x, x_ind, input_x, inputLength);
		for (int i = 0; i < inputLength; i++)
		{
			temp_y[i] = input_y[x_ind[i]];
			temp_w[i] = input_weight[x_ind[i]];
		} //end for		
		std::copy(temp_x, temp_x + inputLength, input_x);
		std::copy(temp_y, temp_y + inputLength, input_y);
		std::copy(temp_w, temp_w + inputLength, input_weight);

	} //end if
	delete[] temp_x;
	delete[] temp_y;
	delete[] temp_w;
	// printf("\nData are ascending, sorting is NOT necessary");
	ArrayMatrixTools::diff(diff_X, input_x, inputLength);
	if (ArrayMatrixTools::any(diff_X, diff_X_length, "E", 0.0) == false)  //Data are perfect, no further calculation is needed, no repetition on x
	{
		std::copy(input_x, input_x + inputLength, output_x);
		std::copy(input_y, input_y + inputLength, output_y);
		std::copy(input_weight, input_weight + inputLength, output_weight);
		*outputLength = inputLength;
		// cout<<"\nData are perfect, no further calculation is needed"<<"\t"<<*outputLength;
		delete[] x_ind;
		delete[] diff_X;
		return;
	}
	printf("\nThere is repetetive value");
	//********************************************************************************
	// from here till end is about the case of repetetive value on x, and necssity of deleting them and replacing them by a good interpolation
	//********************************************************************************

	int *foundOutput;
	foundOutput = new int[inputLength];
	int foundOutputLength = 0;
	ArrayMatrixTools::find(foundOutput, &foundOutputLength, diff_X, diff_X_length, "G", 0.0);
	int *x2, *x3, *nx;
	x2 = new int[foundOutputLength + 1];
	x3 = new int[foundOutputLength + 1];
	nx = new int[foundOutputLength + 1];
	x2[0] = 0;
	for (int i = 0; i < foundOutputLength; i++)
	{
		x2[i + 1] = foundOutput[i] + 1;
	}

	std::copy(x2 + 1, x2 + foundOutputLength + 1, x3);
	x3[foundOutputLength] = inputLength;

	for (int i = 0; i < foundOutputLength + 1; i++)
	{
		nx[i] = x3[i] - x2[i];
	}

	double *ybar;
	ybar = new double[foundOutputLength + 1];
	dlib::matrix<double> sumw, wvec, y_array;
	sumw = dlib::ones_matrix<double>(foundOutputLength + 1, 1) - 1;
	wvec = dlib::ones_matrix<double>(foundOutputLength + 1, 1) - 1;
	y_array = dlib::ones_matrix<double>(1, foundOutputLength + 1) - 1;
	for (int i = 0; i < foundOutputLength + 1; i++)
	{
		ybar[i] = input_y[x2[i]];
		sumw(i, 0) = input_weight[x2[i]];
	}
	int *ind;
	ind = new int[foundOutputLength + 1];
	for (int i = 0; i < foundOutputLength + 1; i++)
	{
		if (nx[i] > 1)     //repeated value
		{
			sumw(i, 0) = 0;
			y_array = dlib::ones_matrix<double>(1, foundOutputLength + 1) - 1;
			for (int j = 1; j <= nx[i]; j++)
			{
				ind[j] = j + (x2[i] - 1);
				wvec(j, 0) = input_weight[ind[j]];
				sumw(i, 0) = sumw(i, 0) + wvec(j, 0);
				y_array(0, j) = input_y[ind[j]];
			}
			ybar[i] = (y_array*wvec) / sumw(i, 0);
		}
		output_x[i] = input_x[x2[i]];
		output_y[i] = ybar[i];
		output_weight[i] = sumw(i, 0);
	}
	
	*outputLength =  foundOutputLength;
	delete[] diff_X;
	
	delete[] foundOutput;
	delete[] x2;
	delete[] x3;
	delete[] nx;
	delete[] ybar;
	delete[] ind;
}

//*******************************************************
void CubicPiecewiseFunction::Fit(PieceweisePolynomial* pp, double *input_x, double *input_y, double *input_w, int inputLength, double smoothingFactor)
{
	/*
	https://wiki.dlr.de/confluence/display/fau/How+to+use+Fit+function
	*/

	//if(inputLength<2)		  printf("\nError, to use fit function, your data must have at least two parameter");
	double *yi, *xi, *wi;

	xi = new double[inputLength];
	yi = new double[inputLength];
	wi = new double[inputLength];
	int n;
	PreProcess(xi, yi, wi, &n, input_x, input_y, input_w, inputLength);
	//std::cout<<"\n"<<n;
	double *dx, *dy, *divdif, *odx;   //*odx-->1/dx
	dx = new double[n];
	odx = new double[n];
	dy = new double[n];
	divdif = new double[n - 1];
	ArrayMatrixTools::diff(dx, xi, n);
	ArrayMatrixTools::diff(dy, yi, n);
	for (int i = 0; i < n - 1; i++)
	{
		odx[i] = 1 / dx[i];
	}

	ArrayMatrixTools::pointwise_multiply(divdif, odx, dy, n - 1);

	double *dx1n_2, *dx0n_3, *dx2x1n_20n_3;
	dx1n_2 = new double[n - 2];
	dx0n_3 = new double[n - 2];
	dx2x1n_20n_3 = new double[n - 2];
	ArrayMatrixTools::pieceOfArray(dx1n_2, dx, 1, n - 2);
	ArrayMatrixTools::pieceOfArray(dx0n_3, dx, 0, n - 3);
	for (int i = 0; i < n - 2; i++) {
		dx2x1n_20n_3[i] = 2 * (dx1n_2[i] + dx0n_3[i]);
	}

	double *sp_dx1n_2, *sp_dx0n_3, *sp_dx2x1n_20n_3;     //sp referes to sparse
	sp_dx1n_2 = new double[(n - 2)*(n - 2)];
	sp_dx0n_3 = new double[(n - 2)*(n - 2)];
	sp_dx2x1n_20n_3 = new double[(n - 2)*(n - 2)];

	ArrayMatrixTools::spdiags(sp_dx1n_2, dx1n_2, n - 2, n - 2, n - 2, -1);
	ArrayMatrixTools::spdiags(sp_dx0n_3, dx0n_3, n - 2, n - 2, n - 2, 1);
	ArrayMatrixTools::spdiags(sp_dx2x1n_20n_3, dx2x1n_20n_3, n - 2, n - 2, n - 2, 0);

	double *R;
	R = new double[(n - 2)*(n - 2)];

	for (int i = 0; i < n - 2; i++)  //data are from 1-->n-2 or 0-->n-3 this is why there is an extrea -2 in i<n-2-2
	{
		for (int j = 0; j < n - 2; j++)
		{
			R[i*(n - 2) + j] = sp_dx1n_2[i*(n - 2) + j] + sp_dx0n_3[i*(n - 2) + j] + sp_dx2x1n_20n_3[i*(n - 2) + j];
		}
	}

	double *odx1n_2 = nullptr, *odx0n_3, *odx_1n_20n_3;
	odx1n_2 = new double[n - 2];
	odx0n_3 = new double[n - 2];
	odx_1n_20n_3 = new double[n - 2];
	ArrayMatrixTools::pieceOfArray(odx1n_2, odx, 1, n - 2);
	ArrayMatrixTools::pieceOfArray(odx0n_3, odx, 0, n - 3);
	for (int i = 0; i < n - 2; i++) {
		odx_1n_20n_3[i] = -1 * (odx1n_2[i] + odx0n_3[i]);
	}


	double *sp_odx1n_2, *sp_odx0n_3, *sp_odx_1n_20n_3;     //sp referes to sparse
	sp_odx1n_2 = new double[(n - 2)*(n)];
	sp_odx0n_3 = new double[(n - 2)*(n)];
	sp_odx_1n_20n_3 = new double[(n - 2)*(n)];

	ArrayMatrixTools::spdiags(sp_odx1n_2, odx1n_2, n - 2, n, n - 2, 2);
	ArrayMatrixTools::spdiags(sp_odx0n_3, odx0n_3, n - 2, n, n - 2, 0);
	ArrayMatrixTools::spdiags(sp_odx_1n_20n_3, odx_1n_20n_3, n - 2, n, n - 2, 1);

	double *Qt, *QtT;
	Qt = new double[(n - 2)*(n)];
	QtT = new double[(n)*(n - 2)];

	for (int i = 0; i < n - 2; i++)  //data are from 1-->n-2 or 0-->n-3 this is why there is an extrea -2 in i<n-2-2
	{
		for (int j = 0; j < n; j++)
		{
			Qt[i*(n)+j] = sp_odx1n_2[i*(n)+j] + sp_odx0n_3[i*(n)+j] + sp_odx_1n_20n_3[i*(n)+j];
		}
	}

	double *ow, *sp_w, *Qtw, *QtWQ, *temp_M;
	ow = new double[n];
	sp_w = new double[n*n];
	Qtw = new double[(n - 2)*n];
	QtWQ = new double[(n - 2)*(n - 2)];
	temp_M = new double[(n - 2)*(n - 2)];
	for (int i = 0; i < n; i++)
	{
		ow[i] = 1 / wi[i];
	}

	ArrayMatrixTools::spdiags(sp_w, ow, n, n, n, 0);
	ArrayMatrixTools::transpose(QtT, Qt, n - 2, n);

	ArrayMatrixTools::matrixMultiplication(Qtw, Qt, n - 2, n, sp_w, n, n);
	ArrayMatrixTools::matrixMultiplication(QtWQ, Qtw, n - 2, n, QtT, n, n - 2);
	dlib::matrix<double> M(n - 2, n - 2);
	dlib::matrix<double> inv_M;
	for (int i = 0; i < (n - 2); i++) {
		for (int j = 0; j < (n - 2); j++) {
			M(i, j) = 6 * (1 - smoothingFactor)*QtWQ[i*(n - 2) + j] + smoothingFactor * R[i*(n - 2) + j];
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

	double *diff_divdif, *u;
	diff_divdif = new double[n - 2];
	u = new double[n - 2];
	ArrayMatrixTools::diff(diff_divdif, divdif, n - 1);
	ArrayMatrixTools::matrixMultiplication(u, temp_invM, n - 2, n - 2, diff_divdif, n - 2, 1);

	double *u_n, *diff_u_n;
	u_n = new double[n]; //in order to add zero at beginning and at the end
	diff_u_n = new double[n - 1];
	u_n[0] = 0;
	u_n[n - 1] = 0;
	for (int i = 0; i < n - 2; i++)
		u_n[i + 1] = u[i];

	ArrayMatrixTools::diff(diff_u_n, u_n, n); //diff([zeros(1,yd); u; zeros(1,yd)])

	double *diff_u_n_odx;
	diff_u_n_odx = new double[n - 1];
	ArrayMatrixTools::pointwise_multiply(diff_u_n_odx, diff_u_n, odx, n - 1); //// diff([zeros(1,yd); u; zeros(1,yd)])./dx(:,dd)

	double *diff_unodx_n1; //to add a zero at beggining and the end (needed for diff)
	diff_unodx_n1 = new double[n + 1];
	diff_unodx_n1[0] = 0;
	diff_unodx_n1[n] = 0;
	for (int i = 0; i < n - 1; i++)
		diff_unodx_n1[i + 1] = diff_u_n_odx[i];

	double *diff2_unodx_n1;
	diff2_unodx_n1 = new double[n];
	ArrayMatrixTools::diff(diff2_unodx_n1, diff_unodx_n1, n + 1);

	for (int i = 0; i < n; i++)
	{
		yi[i] = yi[i] - (6 * (1 - smoothingFactor))*sp_w[i*n + i] * diff2_unodx_n1[i];
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
		c2[i] = diff_yi[i] * odx[i] - dx[i] * (2 * c3[i] + c3[i + 1]);

	double *coef_1, *coef_2, *diff_c3;
	coef_1 = new double[n - 1];
	coef_2 = new double[n - 1];
	diff_c3 = new double[n - 1];
	ArrayMatrixTools::diff(diff_c3, c3, n);


	for (int i = 0; i < n - 1; i++) {
		coef_1[i] = diff_c3[i] * odx[i];
		coef_2[i] = 3 * c3[i];
	}
	//ppCubicSplineDynamicSize(*pp, n-1);
	mkpp(pp, xi, coef_1, coef_2, c2, yi, n - 1);
	//deleteing all dynamic array
	delete[] xi;
	delete[] yi;
	delete[] wi;
	delete[] dx1n_2;
	delete[] dx0n_3;
	delete[] dx2x1n_20n_3;
	delete[] dx;
	delete[] dy;
	delete[] divdif;
	delete[] odx;
	delete[] sp_dx1n_2;
	delete[] sp_dx0n_3;
	delete[] sp_dx2x1n_20n_3;
	delete[] R;
	delete[] odx1n_2;
	delete[] odx0n_3;
	delete[] odx_1n_20n_3;
	delete[] sp_odx1n_2;
	delete[] sp_odx0n_3;
	delete[] sp_odx_1n_20n_3;
	delete[]  Qt;
	delete[] QtT;
	delete[]  temp_invM;
	delete[] ow;
	delete[] sp_w;
	delete[] Qtw;
	delete[] QtWQ;
	delete[] temp_M;
	delete[] diff_u_n_odx;
	delete[] u_n;
	delete[] diff_u_n;
	delete[] diff_divdif;
	delete[] u;
	delete[] diff_unodx_n1;
	delete[] diff2_unodx_n1;
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
		//std::cout<<"\n"<<point<<"\t"<<pp.breaks[0]<<"\t"<<pp.breaks[pp.dimension]<<"\t"<<point-pp.breaks[0];
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