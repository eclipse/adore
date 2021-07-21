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
#include <iostream>
#include <fstream>
#include <string>
#include <valarray>
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "dlib/matrix/matrix.h"
#include "dlib/matrix.h"
#include <adore/mad/arraymatrixtools.h>
#include <vector>
namespace adore
{
	namespace mad
	{
		/**
		*  This class creates smoothend cubic piecewise polynomial
		https://wiki.dlr.de/confluence/display/fau/How+to+use+Fit+function
		*/
		class CubicPiecewiseFunction
		{
		private:
			static inline void repeatedValueInterpolation(double *output_x, double *output_y, double *output_weight,int *outputLength,double *input_x, double *input_y, double *input_weight,int inputLength);	

			struct LocalCoordination
			{
				std::vector<double> lc_breaks;
				std::vector<int> lc_index;
			};			
		
		public:
			/***
			 Pieceweise Polynomial structure
			 */
			struct PieceweisePolynomial  
			{
				double *breaks; 
				double *coef_1; 
				double *coef_2; 
				double *coef_3; 
				double *coef_4; 
				int dimension ; 
			};
			/**
			* generates the break points from a starting point with a given resolution
			* @param Userbreaks is the generated breaks (output)
			* @param UserbreaksLength is the size of Userbreaks (input)
			* @param start is the first break value (input)
			* @param resolution is the breaks resolution (input)
			*/
			static void BreaksGenerator(double *Userbreaks,int UserbreaksLength,double start, double resolution);
			  /**
			   * generates the break points from a starting point to an end point
			   * @param start is the first break value (input)
			   * @param end is the last break value (input)
			   * @param NumberOfPoints is the number of breaks 
			   * @return is breaks
			   */
			static std::vector<double> BreaksGenerator(double start, double end, int NumberOfPoints);
			  /**
			   * generates the break points from a starting point to an end point with exponential resolution
			   * @param Userbreaks is the generated breaks (output)
			   * @param NumberOfPoints is the number of breaks 
			   * @param start is the first break value (input)
			     @param end is the last break value (input)
			   * @return is breaks
			   */
			static void BreaksGeneratorExp(double *Userbreaks, int NumOfPoints, double start, double end);
			  /**
			   * generates smoothend cubinc piecewise polynomial of a data 
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   * @param input_x is data vector (x_axis)
			   * @param input_y is data vector (y_axis)
			   * @param input_w is the weight of data [0, 1]
			   * @param inputLength is the input size
			   * @param smoothingFactor is the smoothing factor [0 , 1]
			   */
			static void smoothingSpline(PieceweisePolynomial* pp, double* input_x, double* input_y, double* input_w, int inputLength,double smoothingFactor);
			  /**
			   * generates smoothend cubinc piecewise polynomial of a data 
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   * @param input_x is data vector (x_axis)
			   * @param input_y is data vector (y_axis)
			   * @param input_w is the weight of data [0, 1]
			   * @param inputLength is the input size
			   * @param smoothingFactor is the smoothing factor [0 , 1]
			   */
			static void fit(PieceweisePolynomial* pp, double* input_x, double* input_y, double* input_w, int inputLength,double smoothingFactor);
			  /**
			   * matches the given breaks to the polynomial breaks
			   * @param Userbreaks is the given breaks
			   * @param UserbreaksLength is the size of Userbreaks
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   */			
			static LocalCoordination localCoordination(double *Userbreaks, int UserbreaksLength, PieceweisePolynomial &pp);
			  /**
			   * evaluates the cubic piecewise polynomial and its first, second and third derivative for a given breaks
			   * @param interpolatedSpline is the interpolation result
			   * @param d_interpolatedSpline is the first derivative of interpolation result
			   * @param dd_interpolatedSpline is the second derivative of interpolation result
			   * @param ddd_interpolatedSpline is the third derivative of interpolation result
			   * @param Userbreaks is the given breaks
			   * @param UserbreaksLength is the size of Userbreaks
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   */
			static void CubicSplineEvaluation( double *interpolatedSpline,double *d_interpolatedSpline,double *dd_interpolatedSpline,double *ddd_interpolatedSpline, double *Userbreaks,int UserbreaksLength,PieceweisePolynomial &pp);
			  /**
			   * evaluates the cubic piecewise polynomial and its first and second derivative for a given breaks
			   * @param interpolatedSpline is the interpolation result
			   * @param d_interpolatedSpline is the first derivative of interpolation result
			   * @param dd_interpolatedSpline is the second derivative of interpolation result
			   * @param Userbreaks is the given breaks
			   * @param UserbreaksLength is the size of Userbreaks
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   */
			static void CubicSplineEvaluation( double *interpolatedSpline,double *d_interpolatedSpline,double *dd_interpolatedSpline, double *Userbreaks,int UserbreaksLength, PieceweisePolynomial &pp);
			  /**
			   * evaluates the cubic piecewise polynomial and its first derivative for a given breaks
			   * @param interpolatedSpline is the interpolation result
			   * @param d_interpolatedSpline is the first derivative of interpolation result
			   * @param Userbreaks is the given breaks
			   * @param UserbreaksLength is the size of Userbreaks
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   */
			static void CubicSplineEvaluation( double *interpolatedSpline,double *d_interpolatedSpline, double *Userbreaks,int UserbreaksLength, PieceweisePolynomial &pp);
			  /**
			   * evaluates the cubic piecewise polynomial for a given breaks 
			   * @param interpolatedSpline is the interpolation result 
			   * @param Userbreaks is the given breaks
			   * @param UserbreaksLength is the size of Userbreaks
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   */
			static void CubicSplineEvaluation( double *interpolatedSpline, double *Userbreaks,int UserbreaksLength, PieceweisePolynomial &pp);
			  /**
			   * makes the piecewise polynomial
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   * @param input_breaks is the given breaks
			   * @param input_coef1 is the given coef_1
			   * @param input_coef2 is the given coef_2
			   * @param input_coef3 is the given coef_3
			   * @param input_coef4 is the given coef_4
			   * @param inputLength is the size of the inputs
			   */			
			static void toPolynomialFrom(PieceweisePolynomial* pp,double *input_breaks,double *input_coef1,double *input_coef2,double *input_coef3,double *input_coef4,int inputLength);	
				/**
			   * returns the index of a point witin breaks of a cubic peicewise polynomial
			   * @param point is the given point
			   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
			   * @return is the index
			   */	
			static int findIndex (double point, PieceweisePolynomial &pp);
				  /**
				   * evaluate a cubic piecewise polynomial at a given point
				   * @param index is the index of a given point
				   * @param point is the given point
				   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
				   * @return is the evaluated value
				   */
			static double splineEvaluation(int index, double point, PieceweisePolynomial &pp );
				  /**
				   * delete a piecewise polynomial structure
				   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
				   */
			static void deleteCubicSplinepp(PieceweisePolynomial& pp);
				  /**
				   * delete a piecewise polynomial structure
				   * @param pp is the structure containing the piecewise polynomial (breaks, coef_1, coef_2, coef_3, coef_4)
				   */
			static void deleteCubicSplinepp1(PieceweisePolynomial* pp);
			

			
		};
	}
}