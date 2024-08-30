
#include "Equation.h"
#include "main.h"
#include "math.h"


void transposeMatrix(int rows, int cols, float matrix[rows][cols], float result[cols][rows]) {
    int i, j;
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            result[j][i] = matrix[i][j];
        }
    }
}

void multiplyMatrices(int row1, int col1, int col2, float mat1[row1][col1], float mat2[col1][col2], float result[row1][col2])
{
	int i, j, k;
    for (i = 0; i < row1; i++) 
		{
			for (j = 0; j < col2; j++) 
			{
				result[i][j] = 0;
				for (k = 0; k < col1; k++) 
				{
						result[i][j] += mat1[i][k] * mat2[k][j];
				}
			}
    }
}

void multiplyMatrixWithScalar(int row, int col, float matrix[row][col], float scalar, float result[row][col]) 
{
	int i, j;
	for (i = 0; i < row; i++) {
			for (j = 0; j < col; j++) {
					result[i][j] = matrix[i][j] * scalar;
			}
	}
}

void addMatrices(int row, int col, float matrix1[][col], float matrix2[][col], float result[row][col]) 
{
    int i, j;
    for (i = 0; i < row; i++) {
        for (j = 0; j < col; j++) {
            result[i][j] = matrix1[i][j] + matrix2[i][j];
        }
    }
}

void subtractMatrices(int row, int col, float matrix1[][col], float matrix2[][col], float result[row][col]) 
{
    int i, j;
    for (i = 0; i < row; i++) {
        for (j = 0; j < col; j++) {
            result[i][j] = matrix1[i][j] - matrix2[i][j];
        }
    }
}

void assignMatrix(int row, int col, float source[][col], float destination[][col]) {
    int i, j;
    for (i = 0; i < row; i++) {
        for (j = 0; j < col; j++) {
            destination[i][j] = source[i][j];
        }
    }
}

float map(float input, float min_input, float max_input, float min_output, float max_output)
{
	float output;
	output = min_output + (input - min_input)/(max_input - min_input) * (max_output-min_output);
	return output;
}
