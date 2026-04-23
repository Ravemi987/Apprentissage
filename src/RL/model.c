#include "model.h"
#include "utils.h"

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdint.h>
#include <math.h>

typedef struct s_rl_env RLEnv;

/*
Définition d'un modèle 
*/
struct s_rl_model {
    // Environnement contenant les données, les fonctions, ...
    RLEnv *userData;

    // Tableau de valeur des états
    float *stateValues;

    // Q-table
    float *QTable;

    // Politique
    int *policy;

    // Config associée au modèle
    RLConfig config;
};

RLModel* RLModelCreate(RLEnv *userData) {
    RLModel *m = malloc(sizeof(struct s_rl_model));
    m->userData = userData;

    m->stateValues = calloc(RLEnvGetNS(userData), sizeof(float));
    m->policy = calloc(RLEnvGetNS(userData), sizeof(int));
    m->QTable = calloc(RLEnvGetNS(userData)* RLEnvGetNA(userData), sizeof(float));

    // On a une config par défaut
    m->config = RLDefaultConfig();

    return m;
};

/*
Libère le modèle.
*/
void RLModelDelete(RLModel **m) {
    if ((*m) == NULL) return;

    free((*m)->stateValues);
    free((*m) ->policy);
    free((*m)->QTable);
    free(*(m));

    (*m) = NULL;
}

/* Setters */

void RLModelSetConfig(RLModel *m, RLConfig cfg) {
    m->config = cfg;
}

/* Getters */

float *RLModelGetStatesValues(RLModel *m) {
    return m->stateValues;
}

int *RLModelGetPolicy(RLModel *m) {
    return m->policy;
}

float *RLModelGetQTable(RLModel *m) {
    return m->QTable;
}

RLConfig* RLModelGetConfig(RLModel *m) {
    return &(m->config);
}

float RLModelGetReward(RLModel *m, RLState s, RLAction a) {
    return RLEnvGetR(m->userData, s.id, a.id);
}

RLState RLModelGetNextState(RLModel *m, RLState s, RLAction a) {
    return RLEnvGetState(m->userData, RLEnvGetTransitionState(m->userData, s.id, a.id));
}

RLAction RLModelGetBestAction(RLModel *m , RLState s) {
    return RLEnvGetAction(m->userData, m->policy[s.id]);
}


/* Printing */

void RLModelPrintStatesValues(RLModel *m) {
    printFloatArray(m->stateValues, RLEnvGetNS(m->userData));
    printf("\n");
}

void RLModelPrintPolicy(RLModel *m) {
    printIntArray(m->policy, RLEnvGetNS(m->userData));
    printf("\n");
}

void RLModelPrintQTable(RLModel *m) {
    printFloatMatrix(m->QTable, RLEnvGetNS(m->userData), RLEnvGetNA(m->userData));
    printf("\n");
}

/* Fonctions statiques */

static int getBestAction(RLModel *m, int state) {
    return arrayMaxIndex(&(m->QTable[state * RLEnvGetNA(m->userData)]), RLEnvGetNA(m->userData));
}

static float getBestNextQValue(RLModel *m, int nextState) {
    return arrayMax(&(m->QTable[nextState * RLEnvGetNA(m->userData)]), RLEnvGetNA(m->userData));
}

static int getQIndex(RLModel *m, int state, int action) {
    return state * RLEnvGetNA(m->userData) + action;
}

/* Algorithmes */

/* ------------------------------- */

void valueIteration(RLModel *m) {
    float epsilon = m->config.epsilon;
    float gamma = m->config.gamma;
    RLEnv *env = m->userData;
    float delta = DBL_MAX;

    while (delta > epsilon) {
        delta = 0;

        for (int s = 0; s < RLEnvGetNS(env); ++s) {
            float oldValue = m->stateValues[s];
            float maxQ = -DBL_MAX;

            for (int a = 0; a < RLEnvGetNA(env); ++a) {
                float q = RLEnvGetR(env, s, a) + gamma * sum(
                    RLEnvGetTransitionArray(env, s, a), m->stateValues,RLEnvGetNS(env)
                );

                if (q > maxQ) {
                    maxQ = q; 
                    m->policy[s] = a;
                }
            }

            m->stateValues[s] = maxQ;
            delta = fmax(delta, fabs(m->stateValues[s] - oldValue));
        }
    }
}

/* ------------------------------- */

void policyEvaluation(RLModel *m, int *policy) {
    float epsilon = m->config.epsilon;
    float gamma = m->config.gamma;
    RLEnv *env = m->userData;
    float delta = DBL_MAX;

    while (delta > epsilon) {
        delta = 0;

        for (int s = 0; s < RLEnvGetNS(env); ++s) {
            float oldValue = m->stateValues[s];

            int a = policy[s];

            m->stateValues[s] = RLEnvGetR(env, s, a) + gamma * sum(
                RLEnvGetTransitionArray(env, s, a), m->stateValues, RLEnvGetNS(env)
            );

            delta = fmax(delta, fabs(m->stateValues[s] - oldValue));
        }
    }
}

bool policyImprovement(RLModel *m, int *policy) {
    float gamma = m->config.gamma;
    RLEnv *env = m->userData;

    bool isPolicyStable = true;

    for (int s = 0; s < RLEnvGetNS(env); ++s) {
        int oldAction = policy[s];

        float maxQ = -DBL_MAX;
        int bestAction = oldAction;

        for (int a = 0; a < RLEnvGetNA(env); ++a) {
            float q = RLEnvGetR(env, s, a) + gamma * sum(
                RLEnvGetTransitionArray(env, s, a), m->stateValues, RLEnvGetNS(env)
            );

            if (q > maxQ + 1e-7) {
                maxQ = q; 
                bestAction = a;
            }
        }

        policy[s] = bestAction;

        if (oldAction != bestAction) isPolicyStable = false;
    }

    return isPolicyStable;
}

void policyIteration(RLModel *m) {
    arrayRandom(m->policy, RLEnvGetNS(m->userData), RLEnvGetNA(m->userData));

    bool isPolicyStable = false;

    while (!isPolicyStable) {
        policyEvaluation(m, m->policy);
        isPolicyStable = policyImprovement(m, m->policy);
    }
}

/* ------------------------------- */

void QLearning(RLModel *m) {
    float alpha = m->config.alpha;
    float gamma = m->config.gamma;
    RLEnv *env = m->userData;

    for (int epoch = 0; epoch < m->config.epochs; ++epoch) {
        int state = 0;

        for (int step = 0; step < m->config.steps; ++step) {
            float r = (float)rand() / (float)RAND_MAX;
            int action;

            if (r < m->config.epsilon) {
                action =  rand() % RLEnvGetNA(env);
            } else {
                action = getBestAction(m, state);
            }

            int nextState = RLEnvGetTransitionState(env, state, action);
            float reward = RLEnvGetR(env, state, action);

            float nextValue = getBestNextQValue(m, nextState);

            m->QTable[getQIndex(m, state, action)] += alpha * (
                reward + (gamma * nextValue) - m->QTable[getQIndex(m, state, action)]
            );

            state = nextState;
        }
    }

    for (int s = 0; s < RLEnvGetNS(env); ++s) {
        m->policy[s] = getBestAction(m, s);
    }
}
