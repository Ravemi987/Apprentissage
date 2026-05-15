#ifndef __UTILS_H__
#define __UTILS_H__

float sum(float *a1, float *a2, int s);

float arrayMax(double *a, int end);

double arrayMaxDouble(double *a, int end);

int arrayMaxIndex(double *a, int end);

void arrayRandom(int *a, int nStates, int nActions);

void printFloatArray(float *a, int s);

void printIntArray(int *a, int s);

void printFloatMatrix(float *a, int nr, int nc);

int readMatricesFromFile(char *filename, float **T, float **R);

double linear(double *X, double *W, double b, int size);

#endif
