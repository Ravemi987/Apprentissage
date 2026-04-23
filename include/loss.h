#ifndef __DL_LOSS_FUNCTION_H__
#define __DL_LOSS_FUNCTION_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct s_dl_loss_function {
    const char *name;

    double (*apply)(double output, double expected);
    double (*derivative)(double output, double expected);
    double (*loss)(double *output, double *expectedOutput, int size);
    double (*globalLoss)(double *outputs, double *expectedOutputs, int rows, int cols);

    void (*derivativeMatrix)(double *res, double *outputs, double *expectedOutputs, int rows, int cols);
    
} LossFunction;

LossFunction *getMeanSquaredErrorLoss();

LossFunction *getCrossEntropyLoss();

#endif
