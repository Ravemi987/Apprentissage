#include <math.h>
#include "loss.h"


static double mseApply(double output, double expectedOutput) {
    double d = output - expectedOutput;
    return d*d;
}


static double mseDerivative(double output, double expectedOutput) {
    return 2 * (output - expectedOutput);
}


static double mseLoss(double *output, double *expectedOutput, int size) {
    double error = 0.0;

    for (int i = 0; i < size; i++) {
        error += mseApply(output[i], expectedOutput[i]);
    }

    return error;
}


static double mseGlobalLoss(double *outputs, double *expectedOutputs, int rows, int cols) {
    double totalError = 0.0;

    for (int i = 0; i < rows; i++) {
        totalError += mseLoss(&outputs[i * cols], &expectedOutputs[i * cols], cols);
    }

    return totalError / rows;
}


void mseDerivativeMatrix(double *res, double *outputs, double *expectedOutputs, int rows, int cols) {
    int size = rows * cols;

    for (int i = 0; i < size; i++) {
        res[i] = mseDerivative(outputs[i], expectedOutputs[i]);
    }
}


LossFunction *getMeanSquaredErrorLoss() {
    LossFunction *l = malloc(sizeof(LossFunction));
    l->name = "mean_squared_error";
    l->apply = mseApply;
    l->derivative = mseDerivative;
    l->loss = mseLoss;
    l->globalLoss = mseGlobalLoss;
    l->derivativeMatrix = mseDerivativeMatrix;
    return l;
}
