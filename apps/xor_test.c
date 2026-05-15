#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "network.h"
#include "layer.h"

// Données d'entrée XOR
double* generateXORInputs() {
    double *inputs = (double*)malloc(4 * 2 * sizeof(double));
    double data[4][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}};
    for (int i = 0; i < 4; i++) {
        inputs[i * 2] = data[i][0];
        inputs[i * 2 + 1] = data[i][1];
    }
    return inputs;
}

// Labels XOR (0 ou 1)
double* generateXORLabels() {
    double *labels = (double*)malloc(4 * sizeof(double));
    double data[4] = {0, 1, 1, 0};
    for (int i = 0; i < 4; i++) labels[i] = data[i];
    return labels;
}

int main() {
    // 2 entrées -> 2 neurones cachés -> 2 neurones de sortie (Classe 0, Classe 1)
    int layerSizes[] = {2, 2, 2};
    int nbSizes = 3;
    int maxBatchSize = 4;

    /* * POIDS INITIAUX MIS À JOUR POUR 2 NEURONES DE SORTIE
     * Couche 1 (2 features -> 2 neurones) : Identique
     * Couche 2 (2 features -> 2 neurones) : 
     * Neurone de sortie 1 (Classe 0) : poids {1.0, 1.0}, biais {-1.0}
     * Neurone de sortie 2 (Classe 1) : poids {0.0, 0.0}, biais {0.0} (neutre)
     */
    double allWeights[] = {
        1.0, 2.0, -3.0, -2.0, // Couche 1
        1.0, 1.0, 0.0, 0.0    // Couche 2 (Poids Neurone 0 et Neurone 1)
    };
    double allBiases[] = {
        -3.0, 1.0,            // Couche 1
        -1.0, 0.0             // Couche 2 (Biais Neurone 0 et Neurone 1)
    };

    // Initialisation du réseau
    // Note: On utilise "softmax" en sortie pour la classification multi-classe
    NeuralNetwork *nn = networkInitWithWeights(
        layerSizes, nbSizes, allWeights, allBiases, 
        "cross_entropy", "sigmoid", "softmax", maxBatchSize
    );

    double *trainInputs = generateXORInputs();
    double *trainLabels = generateXORLabels();

    printf("--- Entraînement XOR (Sortie 2 neurones + Softmax) ---\n");
    // numClasses = 2 pour activer le One-Hot encoding correct
    networkTrain(nn, trainInputs, trainLabels, 4, 2, 1.0, 1000, 4, 1.0E-4);

    printf("\n--- Résultats des prédictions ---\n");
    for (int i = 0; i < 4; i++) {
        double *input = &trainInputs[i * 2];
        
        // Propagation pour voir les probabilités brutes
        double *output = nnForwardPropagation(nn, input, 1);
        
        // Classe prédite (0 ou 1) via ArgMax
        int predClass = nnPredictClass(nn, input);
        
        printf("Entrée: [%.0f, %.0f] -> Probas: [C0: %.4f, C1: %.4f] -> Prédiction: %d (Attendu: %.0f)\n", 
                input[0], input[1], output[0], output[1], predClass, trainLabels[i]);
    }

    printf("\n--- Accuracy Finale ---\n");
    nnDisplayTestAccuracy(nn, trainInputs, trainLabels, 4);

    // Libération de la mémoire
    free(trainInputs);
    free(trainLabels);
    networkDestroy(&nn); 

    return 0;
}
