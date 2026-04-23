#ifndef __DL_ACTIVATION_FUNCTION_H__
#define __DL_ACTIVATION_FUNCTION_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct s_dl_activation_function {
    const char *name;

    // Pointeur vers la fonction scalaire (pour un seul double)
    double (*apply)(double z);
    double (*derivative)(double z);

    // Pointeur vers la fonction matricielle (pour un tableau complet)
    void (*applyMatrix)(double *input, double *output, int rows, int cols);
    void (*derivativeMatrix)(double *input, double *output, int rows, int cols);

} ActivationFunction;

ActivationFunction *getSigmoidActivation();

ActivationFunction *getReLUActivation();

ActivationFunction *getSiLUActivation();

ActivationFunction *getSoftMaxActivation();

#endif
