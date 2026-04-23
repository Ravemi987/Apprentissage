#include <math.h>
#include "activation.h"
#include "utils.h"


static double softmaxApply(double z) {
    return 0;
}


static double softmaxDerivative(double z) {
    return 0;
}


static void softmaxApplyArray(double *z, double *output, int size) {
    double arrMax = arrayMax(z, size);
    double max = arrMax > 0 ? arrMax : 0.01;
    double sum = 0;

    for (int i = 0; i < size; i++) {
        output[i] = exp(z[i] - max);
        sum += output[i];
    }

    for (int i = 0; i < size; i++) {
        output[i] /= sum;
    }
}


static void softmaxDerivativeArray(double *z, double *output, int size) {
    double softmax[size];
    softmaxApplyArray(z, softmax, size);

    for (int i = 0; i < size; i++) {
        output[i] = softmax[i] * (1 - softmax[i]);
    }
}


static void softmaxApplyMatrix(double *input, double *output, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        softmaxApplyArray(&input[i * cols], &output[i * cols], cols);
    }
}


static void softmaxDerivativeMatrix(double *input, double *output, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        softmaxDerivativeArray(&input[i * cols], &output[i * cols], cols);
    }
}


ActivationFunction * getSoftMaxActivation() {
    ActivationFunction *act = malloc(sizeof(ActivationFunction));
    act->name = "softmax";
    act->apply = softmaxApply;
    act->derivative = softmaxDerivative;
    act->applyMatrix = softmaxApplyMatrix;
    act->derivativeMatrix = softmaxDerivativeMatrix;
    return act;
}
