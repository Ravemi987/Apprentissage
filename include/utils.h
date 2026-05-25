#ifndef __UTILS_H__
#define __UTILS_H__

#include <math.h>

float sum(float *a1, float *a2, int s);

double arrayMax(double *a, int end);

double arrayMaxDouble(double *a, int end);

int arrayMaxIndex(double *a, int end);

void arrayRandom(int *a, int nStates, int nActions);

void printFloatArray(float *a, int s);

void printIntArray(int *a, int s);

void printFloatMatrix(float *a, int nr, int nc);

double linear(double *X, double *W, double b, int size);

double clamp(double val, double min_val, double max_val);

#endif
