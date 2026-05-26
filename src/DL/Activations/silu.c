#include <math.h>
#include "activation.h"


static double siluApply(double z) {
    return z / (1.0 + exp(-z));
}


static double siluDerivative(double z) {
    double sig = 1.0 / (1.0 + exp(-z));
    return sig + z * sig * (1.0 - sig);
}


static void applyMatrixDefault(double (*func)(double), double *input, double *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = func(input[i]);
    }
}


static void siluApplyMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(siluApply, input, output, rows * cols);
}


static void siluDerivativeMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(siluDerivative, input, output, rows * cols);
}


ActivationFunction * getSiLUActivation() {
    ActivationFunction *act = malloc(sizeof(ActivationFunction));
    act->name = "silu";
    act->apply = siluApply;
    act->derivative = siluDerivative;
    act->applyMatrix = siluApplyMatrix;
    act->derivativeMatrix = siluDerivativeMatrix;
    return act;
}
