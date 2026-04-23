#include <math.h>
#include "loss.h"


static double EPSILON = 1E-9;


static double crossEntropyApply(double output, double expectedOutput) {
    return -expectedOutput * log(output + EPSILON);
}


static double crossEntropyDerivative(double output, double expectedOutput) {
    if (output == 0 || output == 1) {
        return 0;
    }
    return (-output + expectedOutput) / (output * (output - 1));
}


static double crossEntropyLoss(double *output, double *expectedOutput, int size) {
    double error = 0.0;

    for (int i = 0; i < size; i++) {
        error += crossEntropyApply(output[i], expectedOutput[i]);
    }

    return error;
}


static double crossEntropyGlobalLoss(double *outputs, double *expectedOutputs, int rows, int cols) {
    double totalError = 0.0;

    for (int i = 0; i < rows; i++) {
        totalError += crossEntropyLoss(&outputs[i * cols], &expectedOutputs[i * cols], cols);
    }

    return totalError / rows;
}


void crossEntropyDerivativeMatrix(double *res, double *outputs, double *expectedOutputs, int rows, int cols) {
    int size = rows * cols;

    for (int i = 0; i < size; i++) {
        res[i] = crossEntropyDerivative(outputs[i], expectedOutputs[i]);
    }
}


LossFunction *getCrossEntropyLoss() {
    LossFunction *l = malloc(sizeof(LossFunction));
    l->name = "cross_entropy";
    l->apply = crossEntropyApply;
    l->derivative = crossEntropyDerivative;
    l->loss = crossEntropyLoss;
    l->globalLoss = crossEntropyGlobalLoss;
    l->derivativeMatrix = crossEntropyDerivativeMatrix;
    return l;
}
