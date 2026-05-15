#include <math.h>
#include "activation.h"


static double linearApply(double z) {
    return z;
}


static double linearDerivative(double z) {
    return 1.0;
}


static void applyMatrixDefault(double (*func)(double), double *input, double *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = func(input[i]);
    }
}


static void linearApplyMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(linearApply, input, output, rows * cols);
}


static void linearDerivativeMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(linearDerivative, input, output, rows * cols);
}


ActivationFunction * getLinearActivation() {
    ActivationFunction *act = malloc(sizeof(ActivationFunction));
    act->name = "linear";
    act->apply = linearApply;
    act->derivative = linearDerivative;
    act->applyMatrix = linearApplyMatrix;
    act->derivativeMatrix = linearDerivativeMatrix;
    return act;
}
