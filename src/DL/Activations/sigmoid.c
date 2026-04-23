#include <math.h>
#include "activation.h"


static double sigmoidApply(double z) {
    return 1.0 / (1.0 + exp(-z));
}


static double sigmoidDerivative(double z) {
    double sig = sigmoidApply(z);
    return sig * (1.0 - sig);
}


static void applyMatrixDefault(double (*func)(double), double *input, double *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = func(input[i]);
    }
}


static void sigmoidApplyMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(sigmoidApply, input, output, rows * cols);
}


static void sigmoidDerivativeMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(sigmoidDerivative, input, output, rows * cols);
}


ActivationFunction * getSigmoidActivation() {
    ActivationFunction *act = malloc(sizeof(ActivationFunction));
    act->name = "sigmoid";
    act->apply = sigmoidApply;
    act->derivative = sigmoidDerivative;
    act->applyMatrix = sigmoidApplyMatrix;
    act->derivativeMatrix = sigmoidDerivativeMatrix;
    return act;
}
