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

#include <adore/mad/arraymatrixtools.h>
#include <stdio.h>
#include <math.h>

/**
 *  This class provides mathematical tools for array and matrix. (ex. MAtrix operation)
 */
using namespace std;
using namespace adore::mad;

# define M_PI           3.14159265358979323846  /* pi */

/**
 * computes mean of a vector
 * @param input is vector
 * @param inputSize is the size of vector 
 * @return The mean value
 */
double ArrayMatrixTools::mean(double *input,  int inputSize)
{
	double output = 0.0;
	for (int i = 0; i < inputSize; i++)
	{
		output += input[i];
	}
  return output / inputSize;
}
/**
 * computes sum of a vector
 * @param input is vector
 * @param inputSize is the size of vector
 * @return The sum value
 */
double ArrayMatrixTools::sum(double* input, int inputSize)
{
	double sum = 0;
	for (int i = 0; i < inputSize; i++)
	{
		sum = sum + input[i];
	}
	return sum;
}
/**
 * computes difference between vector elements
 * @param output is vector of difference between input's elements
 * @param input is vector
 * @param inputSize is the size of vector
 */
void ArrayMatrixTools::diff(double* output, double* input, int inputSize)
{
	for (int i = 0; i < inputSize - 1; i++)
	{		
		output[i] = input[i + 1] - input[i];
	}
}
//TODO --> size of matirx (row) can be read directly from Input (Inout->nr())
/**
 * computes difference between vector elements in a matrix (n,1)
 * @param output is vector matrix (n,1) of difference between input's elements
 * @param input is vector matrix (n,1)
 * @param inputSize is the size of vector
 */
void ArrayMatrixTools::diff_Matrix(dlib::matrix<double>& Output, dlib::matrix<double>& Input, int inputSize)
{
  for (int i = 0; i < inputSize - 1; i++)
	{
		Output(i, 0) = Input(i + 1, 0) - Input(i, 0);
	}
}
/**
 * sorts the input data (ascending)
 * @param output sorted data
 * @param outputIndex is the index(location) of the sorted data before being sorted (input)
 * @param inputSize is the size of input vector
 */
void ArrayMatrixTools::sort(double* output, int* outputIndex, double* input, int inputSize)
{
	int tempIndex;
	double *tempInput;
  tempInput = new double[inputSize];
  copy(input, input + inputSize, tempInput);
	double temp;
	//outputIndex initialization
	outputIndex[0] = 0;
  for (int i = 1; i < inputSize; i++)
	{
		outputIndex[i] = outputIndex[i - 1] + 1;
	}
	//sorting
  for (int i = 0; i < inputSize; i++)
	{
    for (int j = inputSize - 1; j > i; j--)
		{
			if (input[j] < input[j - 1])
			{
				temp = input[j - 1];
				tempIndex = outputIndex[j - 1];
				input[j - 1] = input[j];
				outputIndex[j - 1] = outputIndex[j];
				input[j] = temp;
				outputIndex[j] = tempIndex;
			}
		}
	}
  std::memcpy(&output[0], &input[0], inputSize * sizeof(double));
  std::memcpy(&input[0], &tempInput[0], inputSize * sizeof(double));
	delete [] tempInput;
}
/**
 * checks if any data from input elements match the defined condition (equal, bigger, etc. ) with reference 
 * @param return true if the condition is satisfied, else false
 * @param input is the data vector
 * @param inputSize is the size of input vector
 * @param condition is the user condtion as char (E ==, L <, G >, LE <=, GE >= )
 * @param ref is the reference value
 */
bool ArrayMatrixTools::any(double *input, int inputSize, const char* condition, double ref)
{
	bool check = false;
	if (condition[0] == 'E')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] == ref)
			{
				check = true;
				break;
			}
		}
	}
	//*******************************
	else if (condition[0] == 'L')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] < ref)
			{
				check = true;
				break;
			}
		}
	}
	//*******************************
	else if (condition[0] == 'G')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] > ref)
			{
				check = true;
				break;
			}
		}
	}
	//*******************************
	else if (condition[0] == 'L' && condition[1] == 'E')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] <= ref)
			{
				check = true;
				break;
			}
		}
	}
	//*******************************
	else if (condition[0] == 'G' && condition[1] == 'E')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] >= ref)
			{
				check = true;
				break;
			}
		}
	}
	else
	{
		printf("\n %c", condition[0]);
		printf("\nError:(any) Please choose a correct condition");
		printf("\nG for greater, E for equal, L for less");
		printf("\nGE for greater equal, LE for less equal");
	}
	return check;
}
/**
 * finds and return the index of any data from input elements match the defined condition (equal, bigger, etc. ) with reference
 * @param return a vector 
 * @param input is the data vector
 * @param condition is the user condtion as char ( EQUAL, GREATER, LOWER, GREATER_EQUAL, LOWER_EQUAL )
 * @param ref is the reference value
 */
std::vector<int> ArrayMatrixTools::find(std::vector<double> *input, const char *condition, double ref)
{
	std::vector<int> output;
	switch( hashit(condition) )
	{
	case EQUAL:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) == ref)
			{
				output.push_back(i);
			}
		}	

		break;
	case GREATER:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) > ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case LOWER:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) < ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case GREATER_EQUAL:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) >= ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case LOWER_EQUAL:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) <= ref)
			{
				output.push_back(i);
			}
		}	
		break;
	default:   printf("\nNOT VALID CONDITION FOR adore::mad::ArrayMatrixTools::find"); break;
	}
	return output;
}
/**
 * finds and return the index of any data from input elements match the defined condition (equal, bigger, etc. ) with
 * reference
 * @param return a vector
 * @param input is the data vector
 * @param condition is the user condtion as char ( EQUAL, GREATER, LOWER, GREATER_EQUAL, LOWER_EQUAL )
 * @param ref is the reference value
 */
std::vector<int>  ArrayMatrixTools::find(std::vector<int> *input, const char *condition, double ref)
{
	std::vector<int> output;
	switch( hashit(condition) )
	{
	case EQUAL:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) == ref)
			{
				output.push_back(i);
			}
		}	

		break;
	case GREATER:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) > ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case LOWER:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) < ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case GREATER_EQUAL:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) >= ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case LOWER_EQUAL:  
		for(size_t i=0; i<input->size(); i++)
		{
			if (input->at(i) <= ref)
			{
				output.push_back(i);
			}
		}		
		break;
	default:   printf("\nNOT VALID CONDITION FOR adore::mad::ArrayMatrixTools::find"); break;
	}
	return output;
}
/**
 * finds and return the index of any data from input elements match the defined condition (equal, bigger, etc. ) with
 * reference
 * @param return a vector
 * @param input is the data vector
 * @param inputSize is the data vector size
 * @param condition is the user condtion as char ( EQUAL, GREATER, LOWER, GREATER_EQUAL, LOWER_EQUAL )
 * @param ref is the reference value
 */
std::vector<int>  ArrayMatrixTools::find(double *input, int inputSize, const char *condition, double ref)
{
	std::vector<int> output;
	switch( hashit(condition) )
	{
	case EQUAL:  
		for(int i=0; i<inputSize; i++)
		{
			if (input[i] == ref)
			{
				output.push_back(i);
			}
		}	

		break;
	case GREATER:  
		for(int i=0; i<inputSize; i++)
		{
			if (input[i] > ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case LOWER:  
		for(int i=0; i<inputSize; i++)
		{
			if (input[i] < ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case GREATER_EQUAL:  
		for(int i=0; i<inputSize; i++)
		{
			if (input[i] >= ref)
			{
				output.push_back(i);
			}
		}	
		break;
	case LOWER_EQUAL:  
		for(int i=0; i<inputSize; i++)
		{
			if (input[i] <= ref)
			{
				output.push_back(i);
			}
		}		
		break;
	default:   printf("\nNOT VALID CONDITION FOR adore::mad::ArrayMatrixTools::find"); break;
	}
	return output;
}
/**
 * finds and return the index of any data from input elements match the defined condition (equal, bigger, etc. ) with
 * reference
 * @param output a vector that contains the index of the found elements
 * @param outputSize a the size of output vector
 * @param input is the data vector
 * @param inputSize is the data vector size
 * @param condition is the user condtion as char ( E ==, L <, G >, LE <=, GE >=  )
 * @param ref is the reference value
 */
void ArrayMatrixTools::find(int *output, int *outputSize, double *input, int inputSize, const char *condition, double ref)
{
	int *temp;
	temp = new int[inputSize];
	int temp_length = 0;

	if (condition[0] == 'E')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] == ref)
			{
				temp[temp_length] = i;
				temp_length++;
			}
		}
	}

	//**********************************
	else if (condition[0] == 'G')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] > ref)
			{
				temp[temp_length] = i;
				temp_length++;
			}
		}
	}
	//**********************************
	else if (condition[0] == 'L')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] < ref)
			{
				temp[temp_length] = i;
				temp_length++;
			}
		}
	}
	//**********************************
	else if (condition[0] == 'G' && condition[1] == 'E')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] >= ref)
			{
				temp[temp_length] = i;
				temp_length++;
			}
		}
	}
	//**********************************
	else if (condition[0] == 'L' && condition[1] == 'E')
	{
		for (int i = 0; i < inputSize; i++)
		{
			if (input[i] <= ref)
			{
				temp[temp_length] = i;
				temp_length++;
			}
		}
	}
	//**********************************
	else
	{
		printf("\n %c", condition[0]);
		printf("\nError:(find) Please choose a correct condition");
		printf("\nG for greater, E for equal, L for less");
		printf("\nGE for greater equal, LE for less equal");
	}

	copy(temp, temp + temp_length, output);
	*outputSize = temp_length;
	delete[] temp;
}
/**
 * Transpose operation on input natrix
 * @param output the transposed of input matrix
 * @param input is the input matrix
 */
void ArrayMatrixTools::MatrixTranspose(dlib::matrix<double> &Output, dlib::matrix<double> &Input)
{
  for (int i = 0; i < Input.nr(); i++)
	{
    for (int j = 0; j < Input.nc(); j++)
		{
			Output(j, i) = Input(i, j);
		}
	}
}
/**
 * Pointwise multiplication of two vectors with the same size
 * @param output the result of pointwise multiplication
 * @param input_a is the first vector
 * @param input_b is the second vector
 * @param inputSize is the size of input vectors
 */
void ArrayMatrixTools::pointwise_multiply(double *output, double *input_1, double *input_2, int inputSize)
{
	for (int i = 0; i < inputSize; i++)
	{
		output[i] = input_1[i] * input_2[i];
	}
}
/**
 * returns the given piece of a vector
 * @param output is a piece of input
 * @param input is a vector
 * @param start is the index of the piece  
 * @param end is the index of the piece 
 */
void ArrayMatrixTools::pieceOfArray(double *output, double *input, int start, int end)
{
	int counter = 0;
	for (int i = start; i <= end; i++)
	{
		output[counter] = input[i];
		counter++;
	}
}
/**
 * returns a diagonal sparse matrix of input
 * @param sparse is a the sparse matrix
 * @param input is a vector
 * @param output_row is the number of row of sparse matrix
 * @param output_column is the number of column of sparse matrix
 * @param inputsize is the size of input vector
 * @param Startingreference is the diagonal starting reference
 */
void ArrayMatrixTools::spdiags(double *sparse, double *input, int output_row, int output_column, int inputSize, int StartingReference)
{
	int offsetRow = 0;
	int offsetColumn = 0;
	int counter = 0;
	if (StartingReference > 0)
		offsetColumn = StartingReference;
	if (StartingReference < 0)
		offsetRow = abs(StartingReference);

	if (inputSize < min(output_row - offsetRow, output_column - offsetColumn) || output_row - offsetRow == 0 || output_column - offsetColumn == 0)
	{
		printf("\nError, DiagonalSparseMatrix, requested matrix has bigger diagonal than delivered input");
		return;
	}
	if (output_column > output_row)
	{
		for (int i = 0; i < output_row; i++)
		{
			for (int j = 0; j < output_column; j++)
			{
				if (i - offsetRow == j - offsetColumn)
				{
					sparse[i*output_column + j] = input[counter + offsetRow];
					counter++;
				}
				else
				{
					sparse[i*output_column + j] = 0;
				}
			}
		}
	}
	if (output_column <= output_row)
	{
		for (int i = 0; i < output_row; i++)
		{
			for (int j = 0; j < output_column; j++)
			{
				if (i - offsetRow == j - offsetColumn)
				{
					sparse[(i*output_column) + j] = input[counter + offsetColumn];
					counter++;
				}
				else
				{
					sparse[(i*output_column) + j] = 0;
				}
			}
		}
	}
}
/**
 * returns the transpose of a matrix defined as C array
 * @param output is the transposed matrix (C array)
 * @param input matrix (C array)
 * @param input_row is the number of row
 * @param input_column is the number of column 
 */
void ArrayMatrixTools::transpose(double *output, double *input, int input_row, int input_column)
{
	for (int i = 0; i < input_row; i++)
	{
		for (int j = 0; j < input_column; j++)
		{
			output[j*input_row + i] = input[i*input_column + j];
		}
	}
}
/**
 * multiplies to matrix (defined as C array)
 * @param output is the result of multiplication matrix (C array)
 * @param input_1 is input matrix (C array)
 * @param input_1_row is input matrix number of row 
 * @param input_1_column is input matrix number of column 
 * @param input_2 is input matrix (C array)
 * @param input_2_row is input matrix number of row
 * @param input_2_column is input matrix number of column 
 */
void ArrayMatrixTools::matrixMultiplication(double *output, double *input_1, int input_1_row, int input_1_column, double *input_2, int input_2_row, int input_2_column)
{
	if (input_1_column != input_2_row)
		printf("\n Error, invalid matrix size");

	for (int i = 0; i < input_1_row*input_2_column; i++)
		output[i] = 0;   //initialization

	for (int r_1 = 0; r_1 < input_1_row; r_1++)
	{
		for (int c_2 = 0; c_2 < input_2_column; c_2++)
		{
			for (int c_1 = 0; c_1 < input_1_column; c_1++)
			{
				output[(r_1*input_2_column) + c_2] += input_1[(r_1*input_1_column) + c_1] * input_2[(c_1*input_2_column) + c_2];
			}
		}
	}
}
/**
 * normalize an angle [-2*pi , 2*pi]
 * @param x is the input (angle)
 * @param return is the normalized input
 */
 double ArrayMatrixTools::angle_norm(double x)
{
	x = fmod(x + M_PI, 2 * M_PI);
	if (x < 0)
		x += (2 * M_PI);
	return x - M_PI;
}
/**
 * returns an angle which its jump between consecutive angles is lessthan or equal to pi radians
 * @param prev is the input (previous angle)
 * @param now is the input (current angle)
 * @param return is the output
 */
double ArrayMatrixTools::unwrap(double prev, double now)
{
	return prev + angle_norm(now - prev);
}
/**
 * checks if the input is not a number
 * @param x is the input
 * @param return is true if the input is not a number, else false
 */
bool ArrayMatrixTools::isNaN(double x)
{
	if(x !=x)
	{
		return true;
	}
	else
	{
		return false;
	}

}
/**
 * returns the sign of the input
 * @param x is the input
 * @param return is one if the input positive, else minus one
 */
int ArrayMatrixTools::sign(double x)
{
	if(x>=0.0000)
		return 1;
	else
		return -1;
}
/**
 * returns the sign of the input
 * @param x is the input
 * @param return is one if the input positive, else minus one
 */
int ArrayMatrixTools::sign(int x)
{
	if(x>=0)
		return 1;
	else
		return -1;
}
/**
 * returns the mean value of the exponential weigthed input  
 * @param input is the input vector
 * @param inputSize is the size of input 
 * @param return is mean value
 */
double ArrayMatrixTools::exponentialAverage(double *input, int inputSize)
{
	double sum = 0.0;
	double wSum = 0.0;
	double w;
	double pow ;

	for(int i=0; i<inputSize;i++)
	{
		pow =  i-inputSize+1;
		w = std::exp(pow);
		wSum += w;
		sum += w * input[i];
	}
	return sum/wSum;
}
/**
 * returns the average value of the exponential weigthed input (greater weight and significance on the most recent input data)
 * @param input is the input vector
 * @param inputSize is the size of input
 * @param newValue is most recent data
 * @param return is mean value
 */
double ArrayMatrixTools::exponentialMovingAverage(double *input, int inputSize, double newValue)
{
	double *tmp;
	tmp = new double [inputSize];	
	std::memcpy(&tmp[0],&input[1],(inputSize-1)* sizeof(double));
	std::memcpy(&input[0],&tmp[0],(inputSize-1)* sizeof(double));
	input[inputSize-1] = newValue;
	delete [] tmp;
	return exponentialAverage(&input[0], inputSize);
}
double ArrayMatrixTools::aglBetwVecInArcLen(double * v1, double * v2,int dim)
{
	double magV1 = 0;
	double magV2 = 0;
	double dotProd = 0;

	for(int i = 0; i < dim; i++)
	{
		magV1 += v1[i] * v1[i];
		magV2 += v2[i] * v2[i];

		dotProd +=  v1[i] * v2[i];
	}

	magV1 = std::sqrt(magV1);
	magV2 = std::sqrt(magV2);

	return acos(dotProd / (magV1 * magV2));
}
ArrayMatrixTools::CONDITION ArrayMatrixTools::hashit (std::string const& inString) 
{
	if (inString == "E") return EQUAL;
	if (inString == "G") return GREATER;
	if (inString == "L") return LOWER;
	if (inString == "GE") return GREATER_EQUAL;
	if (inString == "LE") return LOWER_EQUAL;
	return LOWER_EQUAL;
}

template <int T>
double ArrayMatrixTools::aglBetwVecInArcLen(dlib::vector<double,T> v1, dlib::vector<double,T> v2)
{
	return acos(v1.dot(v2) / (v1.length() * v2.length()));
}

template double ArrayMatrixTools::aglBetwVecInArcLen<2>(dlib::vector<double,2> v1, dlib::vector<double,2> v2);