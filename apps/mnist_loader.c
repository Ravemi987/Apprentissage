#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Assure-toi d'inclure les headers de ton réseau
#include "layer.h"
#include "network.h"

#define MNIST_PATH "files/MNIST/"

// -------------------------------------------------------------------------
// Utilitaires de lecture de fichiers
// -------------------------------------------------------------------------

// Équivalent de scan32BitsInteger : lit 4 octets et les assemble en Big-Endian
static int readBigEndianInt(FILE *file) {
    unsigned char bytes[4];
    if (fread(bytes, 1, 4, file) != 4) {
        return 0;
    }
    return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}

// Lit les images et les normalise (0.0 à 1.0)
double* readImages(const char *filePath, int *numberOfImages, int *featuresNumber) {
    FILE *file = fopen(filePath, "rb");
    if (!file) {
        fprintf(stderr, "Erreur : Impossible d'ouvrir le fichier %s\n", filePath);
        return NULL;
    }

    (void)readBigEndianInt(file);
    *numberOfImages = readBigEndianInt(file);
    int numberOfRows = readBigEndianInt(file);
    int numberOfColumns = readBigEndianInt(file);

    *featuresNumber = numberOfRows * numberOfColumns;
    int totalPixels = (*numberOfImages) * (*featuresNumber);

    double *images = (double*)malloc(totalPixels * sizeof(double));
    unsigned char *buffer = (unsigned char*)malloc(totalPixels);

    // Lecture de tous les pixels d'un coup (très rapide en C)
    if (fread(buffer, 1, totalPixels, file) != (size_t)totalPixels) {
        fprintf(stderr, "Erreur lors de la lecture des pixels.\n");
    }

    // Normalisation
    for (int i = 0; i < totalPixels; i++) {
        images[i] = (double)buffer[i] / 255.0;
    }

    free(buffer);
    fclose(file);
    return images;
}

// Lit les étiquettes (labels)
double* readLabels(const char *filePath, int *numberOfItems) {
    FILE *file = fopen(filePath, "rb");
    if (!file) {
        fprintf(stderr, "Erreur : Impossible d'ouvrir le fichier %s\n", filePath);
        return NULL;
    }

    (void)readBigEndianInt(file);
    *numberOfItems = readBigEndianInt(file);

    double *labels = (double*)malloc(*numberOfItems * sizeof(double));
    unsigned char *buffer = (unsigned char*)malloc(*numberOfItems);

    if (fread(buffer, 1, *numberOfItems, file) != (size_t)*numberOfItems) {
        fprintf(stderr, "Erreur lors de la lecture des labels.\n");
    }

    for (int i = 0; i < *numberOfItems; i++) {
        labels[i] = (double)buffer[i];
    }

    free(buffer);
    fclose(file);
    return labels;
}

// -------------------------------------------------------------------------
// Helpers de récupération des données
// -------------------------------------------------------------------------

double* getTrainData(int *numImages, int *numFeatures) {
    char path[256];
    snprintf(path, sizeof(path), "%s%s", MNIST_PATH, "train-images.idx3-ubyte");
    return readImages(path, numImages, numFeatures);
}

double* getTrainLabels(int *numLabels) {
    char path[256];
    snprintf(path, sizeof(path), "%s%s", MNIST_PATH, "train-labels.idx1-ubyte");
    return readLabels(path, numLabels);
}

double* getTestData(int *numImages, int *numFeatures) {
    char path[256];
    snprintf(path, sizeof(path), "%s%s", MNIST_PATH, "t10k-images.idx3-ubyte");
    return readImages(path, numImages, numFeatures);
}

double* getTestLabels(int *numLabels) {
    char path[256];
    snprintf(path, sizeof(path), "%s%s", MNIST_PATH, "t10k-labels.idx1-ubyte");
    return readLabels(path, numLabels);
}

// Affiche une image dans le terminal (ASCII art)
void displayImage(double *image, int totalPixels) {
    int imageSize = (int)sqrt(totalPixels);

    for (int i = 0; i < imageSize; i++) {
        for (int j = 0; j < imageSize; j++) {
            double pixel = image[i * imageSize + j];
            printf("%s ", pixel > 0 ? "#" : ".");
        }
        printf("\n");
    }
}

// -------------------------------------------------------------------------
// Point d'entrée principal (Test du réseau)
// -------------------------------------------------------------------------

int main() {
    srand(time(NULL));

    int trainNbImages, trainNbFeatures;
    int testNbImages, testNbFeatures;
    int trainNbLabels, testNbLabels;

    printf("Chargement des donnees MNIST...\n");

    double *trainData = getTrainData(&trainNbImages, &trainNbFeatures);
    double *trainLabels = getTrainLabels(&trainNbLabels);
    double *testData = getTestData(&testNbImages, &testNbFeatures);
    double *testLabels = getTestLabels(&testNbLabels);

    if (!trainData || !trainLabels || !testData || !testLabels) {
        fprintf(stderr, "Erreur : Echec du chargement des donnees. Verifiez le chemin MNIST_PATH.\n");
        return -1;
    }

    printf("trainData: length=%d features=%d\n", trainNbImages, trainNbFeatures);
    printf("testData: length=%d features=%d\n", testNbImages, testNbFeatures);
    printf("trainLabels: length=%d\n", trainNbLabels);
    printf("testLabels: length=%d\n", testNbLabels);

    // Initialisation du réseau
    int layerSizes[] = {784, 100, 10};
    int numLayers = sizeof(layerSizes) / sizeof(layerSizes[0]);
    int batchSize = 64;

    // networkCreate(tailles, nbTailles, loss, hiddenAct, outputAct, maxBatchSize)
    NeuralNetwork *nn = networkCreate(layerSizes, numLayers, "cross_entropy", "sigmoid", "softmax", batchSize);

    if (nn == NULL) {
        fprintf(stderr, "Erreur lors de la creation du reseau.\n");
        return -1;
    }

    // Paramètres d'entraînement
    double learningRate = 1.0;
    int epochs = 5;
    double decay = 1E-7;
    int numClasses = 10;

    printf("\n--- Debut de l'entrainement ---\n");
    
    // nnTrain(nn, inputs, expectedLabels, numSamples, numClasses, learningRate, epochs, batchSize, decay)
    nnTrain(nn, trainData, trainLabels, trainNbImages, numClasses, learningRate, epochs, batchSize, decay);

    printf("\n--- Test du modele ---\n");

    // Affichage de la précision sur le jeu de test
    nnDisplayTestAccuracy(nn, testData, testLabels, testNbImages);

    // Libération de la mémoire
    free(trainData);
    free(trainLabels);
    free(testData);
    free(testLabels);
    
    networkDestroy(&nn);

    return 0;
}
