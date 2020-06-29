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

/*
DaR 2016
*/
#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <adore/mad/cubicpiecewisefunction.h>
#include <adore/mad/coordinateconversion.h>
#include <dlib/matrix/matrix.h>
#include <vector>


namespace adore
{
	namespace mad
	{
		class ArrayMatrixTools
		{
		public:
			enum CONDITION { EQUAL, GREATER, LOWER, GREATER_EQUAL, LOWER_EQUAL, UNDEFINED} ;
			static CONDITION hashit (std::string const& inString) ;
			static double mean(double *input,  int inputSize);
			static double sum(double* input, int inputSize); 
			static void diff(double* output, double* input, int inputSize);
			static void diff_Matrix(dlib::matrix<double>& Output, dlib::matrix<double>& Input, int inputSize);
			static void sort(double* output, int* outputIndex, double* input, int inputSize);
			static bool any(double *input, int inputSize, const char* condition, double ref);
			static void find(int *output, int *outputSize, double *input, int inputSize, const char *condition, double ref);
			static std::vector<int> find( std::vector<double> *input, const char *condition, double ref);
			static std::vector<int> find( std::vector<int> *input, const char *condition, double ref);
			static std::vector<int>  find(double *input, int inputSize, const char *condition, double ref);
			static void MatrixTranspose(dlib::matrix<double> &Output, dlib::matrix<double> &Input);
			static void transpose(double *output, double *input, int input_row, int input_column);
			static void matrixMultiplication(double *output, double *input_1, int input_1_row, int input_1_column, double *input_2, int input_2_row, int input_2_column);
			static void pointwise_multiply(double *output, double *input_1, double *input_2, int inputSize);
			static void pieceOfArray(double *output, double *input, int start, int end);
			static void spdiags(double *sparse, double *input, int output_row, int output_column, int inputSize, int StartingReference);
			static double unwrap(double prev, double now);
			static bool isNaN(double x);
			static int sign(double x);
			static int sign(int x);
			
			static double exponentialAverage(double *input, int inputSize);
			static double exponentialMovingAverage(double *input, int inputSize, double newValue);
			static double aglBetwVecInArcLen(double * v1, double * v2,int dim);
			
			template<int T>
			static double aglBetwVecInArcLen(dlib::vector<double,T> v1, dlib::vector<double,T> v2);
			//private:
			static double angle_norm(double x);
		};
	}
}