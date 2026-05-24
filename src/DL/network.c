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


static Layer *getLastLayer(NeuralNetwork *nn) {
    return nn->layers[nn->nbLayers - 1];
}


static LossFunction *scanLossFunction(char *loss) {
    if (strcmp(loss, "cross_entropy") == 0) return getCrossEntropyLoss();
    else return getMeanSquaredErrorLoss();
}


static void nnInitLayers(NeuralNetwork *nn, int* layerSizes, char* hiddenAct, char* outputAct, int maxBatchSize) {
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

    nnInitLayers(nn, layerSizes, hiddenAct, outputAct, maxBatchSize);

    return nn;
}


static void nnInitLayersWithWeights(NeuralNetwork *nn, int* layerSizes, double* allWeights, double* allBiases,
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
    
    nnInitLayersWithWeights(nn, layerSizes, allWeights, allBiases, hiddenAct, outputAct, maxBatchSize);

    return nn;
}


double *nnForwardPropagation(NeuralNetwork *nn, double *inputs, int batchSize) {
    double *currentInput = inputs;

    for (int i = 0; i < nn->nbLayers; i++) {
        currentInput = layerForwardPropagation(nn->layers[i], currentInput, batchSize);
    }

    return currentInput;
}


static void nnBackPropagation(NeuralNetwork *nn, double *outputs, double *expectedOutputs, int batchSize) {
    Layer *outputLayer = getLastLayer(nn);
    double *currentGrad = layerComputeGradients(
        outputLayer, nn->lossFunction, outputs, expectedOutputs, batchSize
    );

    for (int layer = nn->nbLayers - 2; layer >= 0; layer--) {
        currentGrad = layerBackPropagation(nn->layers[layer], nn->layers[layer + 1], currentGrad, batchSize);
    }
}


static void nnUpdateAllWeights(NeuralNetwork *nn, double learningRate, int datasetSize) {
    for (int layer = 0; layer < nn->nbLayers; layer++) {
        layerUpdateWeights(nn->layers[layer], learningRate, datasetSize);
    }
}


static int nnGetPredictedClass(double *outputs, int numClasses) {
    if (numClasses == 1) {
        return outputs[0] >= 0.5 ? 1 : 0;
    }

    int maxIndex = 0;
    double maxVal = outputs[0];
    for (int i = 1; i < numClasses; i++) {
        if (outputs[i] > maxVal) {
            maxVal = outputs[i];
            maxIndex = i;
        }
    }

    return maxIndex;
}


// static int nnGetCorrectPredictions(double *predictions, double *expectedOutputs, int batchSize, int numClasses) {
//     int correct = 0;
//     for (int i = 0; i < batchSize; i++) {
//         int predClass = nnGetPredictedClass(&predictions[i * numClasses], numClasses);
//         int expectedClass = nnGetPredictedClass(&expectedOutputs[i * numClasses], numClasses);

//         if (predClass == expectedClass) {
//             correct++;
//         }
//     }
//     return correct;
// }


static void nnGradientDescent(NeuralNetwork *nn, double *trainInputs, double *expectedOutputs,
                    int rows, double learningRate, int batchSize) {
    int inputCols = layerGetFeaturesNumber(nn->layers[0]);
    int outputCols = layerGetNeuronsNumber(getLastLayer(nn));

    int batchsNumber = (int)ceil((double)rows / batchSize);
    // double totalLoss = 0.0;
    // int totalCorrect = 0;

    for (int batch = 0; batch < batchsNumber; batch++) {
        int start = batch * batchSize;
        int realBatchSize = (rows < start + batchSize) ? (rows - start) : batchSize;

        double *batchInputs = &trainInputs[start * inputCols];
        double *batchExpected = &expectedOutputs[start * outputCols];

        double *outputsPtr = nnForwardPropagation(nn, batchInputs, realBatchSize);
        
        nnBackPropagation(nn, outputsPtr, batchExpected, realBatchSize);

        // totalLoss += nn->lossFunction->globalLoss(outputsPtr, batchExpected, realBatchSize, outputCols) * realBatchSize;
        // totalCorrect += nnGetCorrectPredictions(outputsPtr, batchExpected, realBatchSize, outputCols); 

        nnUpdateAllWeights(nn, learningRate, realBatchSize);
    }
    //printf("Loss: %.6f - Accuracy: %.2f%%\n", totalLoss / rows, ((double)totalCorrect / rows) * 100.0);
}


static double *oneHotEncode(double *expectedClasses, int numSamples, int numClasses) {
    double *encoded = (double*)calloc(numSamples * numClasses, sizeof(double));

    for (int i = 0; i < numSamples; i++) {
        int expectedIndex = (int)expectedClasses[i];
        encoded[i * numClasses + expectedIndex] = 1.0;
    }

    return encoded;
}


void networkTrainClassifier(NeuralNetwork *nn, double *trainInputs, double *expectedOutput, int numSamples, int numClasses,
            double learningRate, int iterationsNumber, int batchSize, double decay) {
    double *encodedOutput = oneHotEncode(expectedOutput, numSamples, numClasses);
    double initialLr = learningRate;
    double currentLr = initialLr;

    for (int epoch = 0; epoch <= iterationsNumber; epoch++) {
        printf("Epoch %d - ", epoch);
        nnGradientDescent(nn, trainInputs, encodedOutput, numSamples, currentLr, batchSize);
        currentLr = initialLr / (1 + decay * epoch);
        
        fflush(stdout);
    }

    free(encodedOutput);
}


void networkTrain(NeuralNetwork *nn, double *trainInputs, double *expectedOutput, int numSamples,
            double learningRate, int iterationsNumber, int batchSize, double decay) {
    double initialLr = learningRate;
    double currentLr = initialLr;

    for (int epoch = 0; epoch <= iterationsNumber; epoch++) {
        //printf("Epoch %d - ", epoch);
        nnGradientDescent(nn, trainInputs, expectedOutput, numSamples, currentLr, batchSize);
        currentLr = initialLr / (1 + decay * epoch);
        
        fflush(stdout);
    }
}


double *nnPredict(NeuralNetwork *nn, double *inputs, int batchSize) {
    double *predPtr = nnForwardPropagation(nn, inputs, batchSize);
    
    int outputSize = layerGetNeuronsNumber(getLastLayer(nn));
    int totalElements = outputSize * batchSize;
    
    double *pred = malloc(totalElements * sizeof(double));
    
    if (pred != NULL) {
        memcpy(pred, predPtr, totalElements * sizeof(double));
    }
    
    return pred;
}


int nnPredictClass(NeuralNetwork *nn, double *input) {
    double *output = nnForwardPropagation(nn, input, 1);
    int numClasses = layerGetNeuronsNumber(getLastLayer(nn));
    return nnGetPredictedClass(output, numClasses);
}


int *nnPredictAllClasses(NeuralNetwork *nn, double *inputs, int numSamples) {
    int *classes = malloc(numSamples * sizeof(int));
    int inputSize = layerGetFeaturesNumber(nn->layers[0]);

    for (int i = 0; i < numSamples; i++) {
        classes[i] = nnPredictClass(nn, &inputs[i * inputSize]);
    }

    return classes;
}


void nnDisplayTestAccuracy(NeuralNetwork *nn, double *inputs, double *expectedClasses, int numSamples) {
    int correct = 0;
    int inputSize = layerGetFeaturesNumber(nn->layers[0]);

    for (int i = 0; i < numSamples; i++) {
        int predicted = nnPredictClass(nn, &inputs[i * inputSize]);
        if (predicted == (int)expectedClasses[i]) {
            correct++;
        }
    }

    double accuracy = (double)correct / numSamples;
    printf("Accuracy: %.4f (%d/%d)\n", accuracy, correct, numSamples);
}


void networkDestroy(NeuralNetwork **nn) {
    for (int i = 0; i < (*nn)->nbLayers; i++) {
        layerDestroy(&((*nn)->layers[i]));
    }

    free((*nn)->layers);
    free((*nn)->lossFunction);
    free(*nn);
    *nn = NULL;
}


void networkCopyWeights(NeuralNetwork *dest, NeuralNetwork *src) {
    if (dest->nbLayers != src->nbLayers) {
        printf("Erreur: Les réseaux n'ont pas le même nombre de couches.\n");
        return;
    }
    
    for (int i = 0; i < dest->nbLayers; i++) {
        layerCopyWeights(dest->layers[i], src->layers[i]);
    }
}


void networkSave(NeuralNetwork *nn, const char *filepath) {
    FILE *file = fopen(filepath, "w");
    if (!file) {
        printf("Erreur: Impossible d'ouvrir %s pour la sauvegarde.\n", filepath);
        return;
    }

    for (int i = 0; i < nn->nbLayers; i++) {
        layerSave(nn->layers[i], file);
    }

    fclose(file);
    printf("Modele sauvegarde en format TEXTE avec succes dans : %s\n", filepath);
}


void networkLoad(NeuralNetwork *nn, const char *filepath) {
    FILE *file = fopen(filepath, "r");
    if (!file) {
        printf("Erreur: Impossible d'ouvrir %s pour le chargement.\n", filepath);
        return;
    }

    for (int i = 0; i < nn->nbLayers; i++) {
        layerLoad(nn->layers[i], file);
    }

    fclose(file);
    printf("Modele charge avec succes depuis : %s\n", filepath);
}
