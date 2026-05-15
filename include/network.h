#ifndef __DL_NETWORK_H__
#define __DL_NETWORK_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "utils.h"
#include "activation.h"
#include "loss.h"
#include "layer.h"


typedef struct s_dl_neural_network NeuralNetwork;


// Alloue de la mémoire !
NeuralNetwork *networkCreate(int* layerSizes, int nbSizes, char* loss, char* hiddenAct, char* outputAct, int maxBatchSize);

// Alloue de la mémoire !
NeuralNetwork *networkInitWithWeights(int* layerSizes, int nbSizes, double* allWeights, double* allBiases,
                                    char* loss, char* hiddenAct, char* outputAct, int maxBatchSize);

double *nnForwardPropagation(NeuralNetwork *nn, double *inputs, int batchSize);

void nnTrain(NeuralNetwork *nn, double *trainInputs, double *expectedOutput, int numSamples, int numClasses,
            double learningRate, int iterationsNumber, int batchSize, double decay);

// Alloue de la mémoire !
double *nnPredict(NeuralNetwork *nn, double *inputs, int batchSize);

int nnPredictClass(NeuralNetwork *nn, double *input);

// Alloue de la mémoire !
int *nnPredictAllClasses(NeuralNetwork *nn, double *inputs, int numSamples);

void nnDisplayTestAccuracy(NeuralNetwork *nn, double *inputs, double *expectedClasses, int numSamples);

void networkDestroy(NeuralNetwork **nn);

#endif
