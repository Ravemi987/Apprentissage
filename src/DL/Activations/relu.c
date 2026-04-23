#include <math.h>
#include "activation.h"


static double reluApply(double z) {
    return z > 0 ? z : 0;
}


static double reluDerivative(double z) {
    return z > 0 ? 1 : 0;
}


static void applyMatrixDefault(double (*func)(double), double *input, double *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = func(input[i]);
    }
}


static void reluApplyMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(reluApply, input, output, rows * cols);
}


static void reluDerivativeMatrix(double *input, double *output, int rows, int cols) {
    applyMatrixDefault(reluDerivative, input, output, rows * cols);
}


ActivationFunction * getReLUActivation() {
    ActivationFunction *act = malloc(sizeof(ActivationFunction));
    act->name = "relu";
    act->apply = reluApply;
    act->derivative = reluDerivative;
    act->applyMatrix = reluApplyMatrix;
    act->derivativeMatrix = reluDerivativeMatrix;
    return act;
}
