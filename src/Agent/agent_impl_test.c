#include "agent_impl_test.h"


typedef void (*ActionFunc)(void *context);


void moveLeft(void *args) {
    if (args) return;

    printf("Déplacement vers la gauche\n");
}

void moveUp(void *args) {
    if (args) return;

    printf("Saut vers le haut\n");
}

void moveRight(void *args) {
    if (args) return;

    printf("Déplacement à droite\n");
}

void eat(void *args) {
    if (args) return;

    printf("Ramassage du bonbon\n");
}

int generateSimpleMatrices(RLState *sArray, RLAction*aArray,
                                   float **T, float **R) {
    if (sArray == NULL || aArray == NULL) return -1;

    return readMatricesFromFile("files/matrices.txt", T, R);
}
