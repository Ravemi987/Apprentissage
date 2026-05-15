#ifndef __DL_LAYER_H__
#define __DL_LAYER_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <omp.h>

#include "utils.h"
#include "activation.h"
#include "loss.h"

typedef struct s_dl_layer Layer;

// Alloue de la mémoire !
Layer * layerCreate(int nbFeatures, int nbNeurons, char *activationFun, int maxBatchSize);

// Alloue de la mémoire !
Layer *layerInitWithWeights(double *initialWeights, double *initialBiases, int nbFeatures, int nbNeurons, char *activationFun, int maxBatchSize);

double *layerGetWeights(Layer *l);

double *layerGetBiases(Layer *l);

int layerGetFeaturesNumber(Layer *l);

int layerGetNeuronsNumber(Layer *l);

double *layerForwardPropagation(Layer *l, double *inputs, int batchSize);

double *layerComputeGradients(Layer *l, LossFunction *lf, double *outputs, double *expectedOutputs, int batchSize);

double *layerBackPropagation(Layer *l, Layer *nextLayer, double *nextGradients, int batchSize);

void layerUpdateWeights(Layer *l, double learningRate, int datasetSize);

void layerDestroy(Layer **l);

void layerCopyWeights(Layer *dest, Layer *src);

void layerSave(Layer *l, FILE *file);

void layerLoad(Layer *l, FILE *file);

#endif
