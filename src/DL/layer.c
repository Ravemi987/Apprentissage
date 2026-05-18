#include "layer.h"


struct s_dl_layer {
    int featuresNumber;
    int neuronsNumber;
    int maxBatchSize;

    // Buffers Forward
    double *activationsBuffer;   // maxBatchSize * featuresNumber
    double *linearInputs;       // maxBatchSize * neuronsNumber
    double *outputsBuffer;     // maxBatchSize * neuronsNumber

    // Buffers Backward
    double *activationDerivatives; // maxBatchSize * neuronsNumber
    double *layerGradients;        // maxBatchSize * neuronsNumber

    // Paramètres
    double *weights;            // neuronsNumber * featuresNumber
    double *biases;             // neuronsNumber

    // Gradients des paramètres
    double *weightsGradients;   // neuronsNumber * featuresNumber
    double *biasesGradients;    // neuronsNumber

    ActivationFunction *activationFunction;
};


static void scanActivationFunction(Layer *l, char *activationFun) {
    if (strcmp(activationFun, "sigmoid") == 0) l->activationFunction = getSigmoidActivation();
    else if (strcmp(activationFun, "relu") == 0) l->activationFunction = getReLUActivation();
    else if (strcmp(activationFun, "silu") == 0) l->activationFunction = getSiLUActivation();
    else if (strcmp(activationFun, "softmax") == 0) l->activationFunction = getSoftMaxActivation();
    else l->activationFunction = getLinearActivation();
}


// On peut faire un memcpy
static void saveActivations(Layer *l, double *inputs, int batchSize) {
    for (int batch = 0; batch < batchSize; ++batch) {
        for (int neuron = 0; neuron < l->featuresNumber; ++neuron) {
            l->activationsBuffer[batch * l->featuresNumber + neuron] = inputs[batch * l->featuresNumber + neuron];
        }
    }
}


// Initialisation de He
static void initWeights(Layer *l) {
    double limit = sqrt(6.0 / (double)l->featuresNumber);

    for (int neuron = 0; neuron < l->neuronsNumber; ++neuron) {
        l->biases[neuron] = 0.0; 

        for (int feature = 0; feature < l->featuresNumber; ++feature) {
            double rand_val = ((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
            l->weights[neuron * l->featuresNumber + feature] = rand_val * limit;
        }
    }
}


double *layerGetWeights(Layer *l) {
    return l->weights;
}


double *layerGetBiases(Layer *l) {
    return l->biases;
}


int layerGetFeaturesNumber(Layer *l) {
    return l->featuresNumber;
}


int layerGetNeuronsNumber(Layer *l) {
    return l->neuronsNumber;
}


Layer *layerCreate(int nbFeatures, int nbNeurons, char *activationFun, int maxBatchSize) {
    Layer *l = (Layer *)malloc(sizeof(struct s_dl_layer));
    if (l == NULL) return NULL;

    l->maxBatchSize = maxBatchSize;
    l->featuresNumber = nbFeatures;
    l->neuronsNumber = nbNeurons;

    scanActivationFunction(l, activationFun);

    l->activationsBuffer = malloc(maxBatchSize * nbFeatures * sizeof(double));
    l->linearInputs = malloc(maxBatchSize * nbNeurons * sizeof(double));
    l->outputsBuffer = malloc(maxBatchSize * nbNeurons * sizeof(double));

    l->activationDerivatives = malloc(maxBatchSize * nbNeurons * sizeof(double));
    l->layerGradients = malloc(maxBatchSize * nbNeurons * sizeof(double));

    l->weights = (double *)malloc(nbNeurons * nbFeatures * sizeof(double));
    l->biases = (double *)malloc(nbNeurons * sizeof(double));

    l->weightsGradients = (double *)calloc(nbNeurons * nbFeatures, sizeof(double));
    l->biasesGradients = (double *)calloc(nbNeurons, sizeof(double));

    initWeights(l);

    return l;
}


Layer *layerInitWithWeights(double *initialWeights, double *initialBiases, int nbFeatures, int nbNeurons, 
                char *activationFun, int maxBatchSize) {
    Layer *l = (Layer *)malloc(sizeof(struct s_dl_layer));
    if (l == NULL) return NULL;
    scanActivationFunction(l, activationFun);
    
    l->maxBatchSize = maxBatchSize;
    l->featuresNumber = nbFeatures;
    l->neuronsNumber = nbNeurons;
    
    l->weights = malloc(nbNeurons * nbFeatures * sizeof(double));
    memcpy(l->weights, initialWeights, nbFeatures * nbNeurons * sizeof(double));

    l->biases = malloc(nbNeurons * sizeof(double));
    memcpy(l->biases, initialBiases, nbNeurons * sizeof(double));

    l->activationsBuffer = malloc(maxBatchSize * nbFeatures * sizeof(double));
    l->linearInputs = malloc(maxBatchSize * nbNeurons * sizeof(double));
    l->outputsBuffer = malloc(maxBatchSize * nbNeurons * sizeof(double));

    l->activationDerivatives = malloc(maxBatchSize * nbNeurons * sizeof(double));
    l->layerGradients = malloc(maxBatchSize * nbNeurons * sizeof(double));

    l->weightsGradients = (double *)calloc(nbNeurons * nbFeatures, sizeof(double));
    l->biasesGradients = (double *)calloc(nbNeurons, sizeof(double));

    return l;
}


static void linearCombination(Layer *l, double *inputs, int batchSize) {
    #pragma omp parallel for collapse(2)
    for (int batch = 0; batch < batchSize; ++batch) {
        for (int neuron = 0; neuron < l->neuronsNumber; ++neuron) {
            l->linearInputs[batch * l->neuronsNumber + neuron] = linear(
                &inputs[batch * l->featuresNumber], 
                &l->weights[neuron * l->featuresNumber], 
                l->biases[neuron], 
                l->featuresNumber
            );
        }
    }
}


double *layerForwardPropagation(Layer *l, double *inputs, int batchSize) {
    saveActivations(l, inputs, batchSize);
    linearCombination(l, inputs, batchSize);

    l->activationFunction->applyMatrix(l->linearInputs, l->outputsBuffer, batchSize, l->neuronsNumber);

    return l->outputsBuffer;
}


static void updateWeightsGradients(Layer *l, int neuron, double nextGradientValue, int batchIndex) {
    for (int feature = 0; feature < l->featuresNumber; feature++) {
        l->weightsGradients[neuron * l->featuresNumber + feature] += 
        l->activationsBuffer[batchIndex * l->featuresNumber + feature] * 
        nextGradientValue;
    }
}


double *layerComputeGradients(Layer *l, LossFunction *lf, double *outputs, double *expectedOutputs, int batchSize) {
    l->activationFunction->derivativeMatrix(l->linearInputs, l->activationDerivatives, batchSize, l->neuronsNumber);
    lf->derivativeMatrix(l->layerGradients, outputs, expectedOutputs, batchSize, l->neuronsNumber);

    // On inverse les boucles
    #pragma omp parallel for
    for (int neuron = 0; neuron < l->neuronsNumber; neuron++) {
        for (int batch = 0; batch < batchSize; batch++) {
            int index = batch * l->neuronsNumber + neuron;

            l->layerGradients[index] = l->layerGradients[index] * l->activationDerivatives[index];
            updateWeightsGradients(l, neuron, l->layerGradients[index], batch);
            l->biasesGradients[neuron] += l->layerGradients[index];
        }
    }

    return l->layerGradients;
}


double *layerBackPropagation(Layer *l, Layer *nextLayer, double *nextGradients, int batchSize) {
    l->activationFunction->derivativeMatrix(l->linearInputs, l->activationDerivatives, batchSize, l->neuronsNumber);

    // On inverse les boucles
    #pragma omp parallel for
    for (int neuron = 0; neuron < l->neuronsNumber; neuron++ ) {
        for (int batch = 0; batch < batchSize; batch++) {
            double currentGrad = 0.0;

            for (int nextNeuron = 0; nextNeuron < nextLayer->neuronsNumber; nextNeuron++) {
                int weightIndex = nextNeuron * nextLayer->featuresNumber + neuron;
                double connectionWeight = nextLayer->weights[weightIndex];

                int nexGradIndex =  batch * nextLayer->neuronsNumber + nextNeuron;
                currentGrad += connectionWeight * nextGradients[nexGradIndex];
            }

            int index =  batch * l->neuronsNumber + neuron;
            l->layerGradients[index] = currentGrad * l->activationDerivatives[index];

            updateWeightsGradients(l, neuron, l->layerGradients[index], batch);
            
            l->biasesGradients[neuron] += l->layerGradients[index];
        }
    }

    return l->layerGradients;
}


void layerUpdateWeights(Layer *l, double learningRate, int datasetSize) {
    #pragma omp parallel for collapse(2)
    for (int neuron = 0; neuron < l->neuronsNumber; neuron++) {
        for (int feature = 0; feature < l->featuresNumber; feature++) {
            int index = neuron * l->featuresNumber + feature;

        double grad = l->weightsGradients[index] / datasetSize;
            
            if (grad > CLIP_LIMIT) grad = CLIP_LIMIT;
            if (grad < -CLIP_LIMIT) grad = -CLIP_LIMIT;

            l->weights[index] -= learningRate * grad;
            l->weightsGradients[index] = 0.0;
        }
    }
    // On separe les boucles
    #pragma omp parallel for    
    for (int neuron = 0; neuron < l->neuronsNumber; neuron++) {
    double bias_grad = l->biasesGradients[neuron] / datasetSize;
        
        if (bias_grad > CLIP_LIMIT) bias_grad = CLIP_LIMIT;
        if (bias_grad < -CLIP_LIMIT) bias_grad = -CLIP_LIMIT;

        l->biases[neuron] -= learningRate * bias_grad;
        l->biasesGradients[neuron] = 0.0;
    }
}


void layerDestroy(Layer **l) {
    free((*l)->activationFunction);
    free((*l)->activationsBuffer);
    free((*l)->linearInputs);
    free((*l)->outputsBuffer);
    free((*l)->activationDerivatives);
    free((*l)->layerGradients);
    free((*l)->weights );
    free((*l)->biases);
    free((*l)->weightsGradients);
    free((*l)->biasesGradients);
    *l = NULL;
}


void layerCopyWeights(Layer *dest, Layer *src) {
    if (dest->featuresNumber != src->featuresNumber || dest->neuronsNumber != src->neuronsNumber) {
        printf("Erreur: Dimensions des couches incompatibles pour la copie.\n");
        return;
    }
    
    int nbWeights = dest->featuresNumber * dest->neuronsNumber;
    memcpy(dest->weights, src->weights, nbWeights * sizeof(double));
    memcpy(dest->biases, src->biases, dest->neuronsNumber * sizeof(double));
}


void layerSave(Layer *l, FILE *file) {
    int nbWeights = l->featuresNumber * l->neuronsNumber;
    
    for (int i = 0; i < nbWeights; i++) {
        fprintf(file, "%.8f ", l->weights[i]);
    }
    fprintf(file, "\n");

    for (int i = 0; i < l->neuronsNumber; i++) {
        fprintf(file, "%.8f ", l->biases[i]);
    }
    fprintf(file, "\n");
}


void layerLoad(Layer *l, FILE *file) {
    int nbWeights = l->featuresNumber * l->neuronsNumber;
    
    for (int i = 0; i < nbWeights; i++) {
        if (fscanf(file, "%lf", &l->weights[i]) != 1) {}
    }
    
    for (int i = 0; i < l->neuronsNumber; i++) {
        if (fscanf(file, "%lf", &l->biases[i]) != 1) {}
    }
}
