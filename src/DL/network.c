#include "network.h"
#include "layer.h"


struct s_dl_neural_network {
    Layer **layers;             // Tableau de pointeurs
    int *layerSizes;
    int nbLayers;

    LossFunction *lossFunction;

    char *lossName;
    char *hiddenActivationName;
    char *outputActivationName;

    int maxBatchSize;
};


int *getLayerSizes(NeuralNetwork *n) {
    return n->layerSizes;
}


char *getLoss(NeuralNetwork *n) {
    return n->lossName;
}


char *getHiddenActivation(NeuralNetwork *n) {
    return n->hiddenActivationName;
}


char *getOutputActivation(NeuralNetwork *n) {
    return n->outputActivationName;
}


LossFunction *getLossFunction(NeuralNetwork *n) {
    return n->lossFunction;
}


static LossFunction *scanLossFunction(char *loss) {
    if (strcmp(loss, "cross_entropy") == 0) return getCrossEntropyLoss();
    else return getMeanSquaredErrorLoss();
}


static void networkInitLayers(NeuralNetwork *nn, int* layerSizes, char* hiddenAct, char* outputAct, int maxBatchSize) {
    for (int i = 0; i < nn->nbLayers; i++) {
        char *activation = (i < nn->nbLayers - 1) ? hiddenAct : outputAct;
        nn->layers[i] = layerCreate(layerSizes[i], layerSizes[i+1], activation, maxBatchSize);
    }

}


NeuralNetwork *networkCreate(int* layerSizes, int nbSizes, char* loss, char* hiddenAct, char* outputAct, int maxBatchSize) {
    NeuralNetwork *nn = malloc(sizeof(NeuralNetwork));
    nn->nbLayers = nbSizes - 1;
    nn->layers = malloc(nn->nbLayers * sizeof(Layer *));
    nn->layerSizes = layerSizes;
    nn->lossFunction = scanLossFunction(loss);
    nn->lossName = loss;
    nn->hiddenActivationName = hiddenAct;
    nn->outputActivationName = outputAct;
    nn->maxBatchSize = maxBatchSize;

    networkInitLayers(nn, layerSizes, hiddenAct, outputAct, maxBatchSize);

    return nn;
}


static void networkInitLayersWithWeights(NeuralNetwork *nn, int* layerSizes, double* allWeights, double* allBiases,
                                        char* hiddenAct, char* outputAct, int maxBatchSize) {
    int weightOffset = 0;
    int biasOffset = 0;

    for (int i = 0; i < nn->nbLayers; i++) {
        int nbFeatures = layerSizes[i];
        int nbNeurons = layerSizes[i+1];
        int nbWeights = nbFeatures * nbNeurons;

        char *activation = (i < nn->nbLayers - 1) ? hiddenAct : outputAct;

        nn->layers[i] = layerInitWithWeights(
            &allWeights[weightOffset],
            &allBiases[biasOffset],
            nbFeatures, nbNeurons, activation, maxBatchSize
        );

        weightOffset += nbWeights;
        biasOffset += nbNeurons;
    }
}


NeuralNetwork *networkInitWithWeights(int* layerSizes, int nbSizes, double* allWeights, double* allBiases,
                                    char* loss, char* hiddenAct, char* outputAct, int maxBatchSize) {
    NeuralNetwork *nn = malloc(sizeof(NeuralNetwork));
    nn->nbLayers = nbSizes - 1;
    nn->layers = malloc(nn->nbLayers * sizeof(Layer *));
    nn->layerSizes = layerSizes;
    nn->lossFunction = scanLossFunction(loss);
    nn->lossName = loss;
    nn->hiddenActivationName = hiddenAct;
    nn->outputActivationName = outputAct;
    nn->maxBatchSize = maxBatchSize;
    
    networkInitLayersWithWeights(nn, layerSizes, allWeights, allBiases, hiddenAct, outputAct, maxBatchSize);

    return nn;
}


double *nnForwardPropagation(NeuralNetwork *nn, double *inputs, int batchSize) {
    double *currentInput = inputs;

    for (int i = 0; i < nn->nbLayers; i++) {
        currentInput = layerForwardPropagation(nn->layers[i], currentInput, batchSize);
    }

    return currentInput;
}


void nnBackPropagation(NeuralNetwork *nn, double *outputs, double *expectedOutputs, int batchSize) {
    Layer *outputLayer = nn->layers[nn->nbLayers - 1];
    double *currentGrad = layerComputeGradients(
        outputLayer, nn->lossFunction, outputs, expectedOutputs, batchSize
    );

    for (int layer = nn->nbLayers - 2; layer >= 0; layer--) {
        currentGrad = layerBackPropagation(nn->layers[layer], nn->layers[layer + 1], currentGrad, batchSize);
    }
}


void nnUpdateAllWeights(NeuralNetwork *nn, double learningRate, int datasetSize) {
    for (int layer = 0; layer < nn->nbLayers; layer++) {
        layerUpdateWeights(nn->layers[layer], learningRate, datasetSize);
    }
}


