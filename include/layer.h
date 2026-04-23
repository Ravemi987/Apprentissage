#ifndef __DL_LAYER_H__
#define __DL_LAYER_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "utils.h"
#include "activation.h"
#include "loss.h"

typedef struct s_dl_layer Layer;

Layer * createLayer(int nbFeatures, int nbNeurons, char *activationFun, int maxBatchSize);

Layer *initLayer(double *initialWeights, double *initialBiases, int nbFeatures, int nbNeurons, char *activationFun, int maxBatchSize);

double *getWeights(Layer *l);

double *getBiases(Layer *l);

int getFeaturesNumber(Layer *l);

int getNeuronsNumber(Layer *l);

double *layerForwardPropagation(Layer *l, double *inputs);

double *layerForwardPropagationBatch(Layer *l, double *inputs, int batchSize);

double *layerComputeGradientsBatch(Layer *l, LossFunction *lf, double *outputs, double *expectedOutputs, int batchSize);

double *layerBackPropagationBatch(Layer *l, Layer *nextLayer, double *nextGradients, int batchSize);

void layerUpdateWeights(Layer *l, double learningRate, int datasetSize);

#endif
