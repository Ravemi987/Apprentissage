#include <stdio.h>
#include <stdlib.h>

int random_state = 1804289383;

/*
Somme des produits terme à terme de deux tableaux 1D
*/
float sum(float *a1, float *a2, int s) {
    float r = 0;

    for (int i = 0; i < s; i++) {
        r += a1[i] * a2[i];
    }

    return r;
}

float arrayMax(float *a, int n) {
    float m = a[0];
    for (int i = 1; i < n; ++i) {
        if (a[i] > m) m = a[i];
    }
    return m;
}

double arrayMaxDouble(double *a, int n) {
    double m = a[0];
    for (int i = 1; i < n; ++i) {
        if (a[i] > m) m = a[i];
    }
    return m;
}

int arrayMaxIndex(float *a, int n) {
    float m = a[0];
    int index = 0;
    for (int i = 1; i < n; ++i) {
        if (a[i] > m) {
            m = a[i];
            index = i;
        }
    }
    return index;
}

void arrayRandom(int *a, int nStates, int nActions) {
    for (int i = 0; i < nStates; ++i) {
        a[i] = rand() % nActions;
    }
}

void printFloatArray(float *a, int s) {
    for (int i = 0; i < s; ++i) {
        if (i < s-1) {
            printf("%f, ", a[i]);
        } else {
            printf("%f\n", a[i]);
        }
        
    }
}

void printIntArray(int *a, int s) {
    for (int i = 0; i < s; ++i) {
        if (i < s-1) {
            printf("%d, ", a[i]);
        } else {
            printf("%d\n", a[i]);
        }
        
    }
}

void printFloatMatrix(float *a, int nr, int nc) {
    for (int r = 0; r < nr; ++r) {
        for (int c = 0; c < nc; ++c) {
            printf("%8.2f ", a[r * nc + c]);
        }
        printf("\n");
    }
}

int readMatricesFromFile(char *filename, float **T, float **R) {
    FILE *f = fopen(filename, "r");
    if (!f) return -1;

    int nS, nA;
    if (fscanf(f, "%d %d", &nS, &nA) != 2) {fclose(f); return -1;}

    int tSize = nS * nA * nS;
    int rSize = nS * nA;

    *T = calloc(tSize, sizeof(float));
    *R = calloc(rSize, sizeof(float));

    if (*T == NULL || *R == NULL) {fclose(f); return -1;}

    for (int i = 0; i < tSize; ++i) {
        if (fscanf(f, "%f", &((*T)[i])) == 0) return -1;
    }

    for (int i = 0; i < rSize; ++i) {
        if (fscanf(f, "%f", &((*R)[i])) == 0) return -1;
    } 

    fclose(f);

    return 0;
}

double linear(double *X, double *W, double b, int size) {
    double r = 0;

    for (int i = 0; i < size; ++i) {
        r += X[i] * W[i];
    }

    return r + b;
}
