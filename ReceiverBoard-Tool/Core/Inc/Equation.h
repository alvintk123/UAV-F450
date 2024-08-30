#ifndef _EQUATION_H_
#define _EQUATION_H_
#include <stdio.h>


void transposeMatrix(int rows, int cols, float matrix[rows][cols], float result[cols][rows]);
void multiplyMatrices(int row1, int col1, int col2, float mat1[row1][col1], float mat2[col1][col2], float result[row1][col2]);
void multiplyMatrixWithScalar(int row, int col, float matrix[row][col], float scalar, float result[row][col]);
void addMatrices(int row, int col, float matrix1[][col], float matrix2[][col], float result[row][col]);
void subtractMatrices(int row, int col, float matrix1[][col], float matrix2[][col], float result[row][col]);
void assignMatrix(int row, int col, float source[][col], float destination[][col]);
float map(float input, float min_input, float max_input, float min_output, float max_output);

#endif
