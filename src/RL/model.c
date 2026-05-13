#include "model.h"
#include "utils.h"

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdint.h>
#include <math.h>


static void initNetwork(DQNModel *m) {
    // int layerSizes[] = 
    // m->q_network = networkCreate([])
}


DQNModel* DQNModelCreate() {
    DQNModel *m = malloc(sizeof(struct s_rl_model));

    initNetworks(m);
    initReplayBuffer(m);
    m->config = defaultConfig();
    m->train_step_count = 0;
    m->target_update_freq = 1000;

    return m;
};

/*
Libère le modèle.
*/
void DQNModelDelete(DQNModel **m) {
    if ((*m) == NULL) return;

    free(*(m));

    (*m) = NULL;
}

/* Setters */

void DQNModelSetConfig(DQNModel *m, Config cfg) {
    m->config = cfg;
}

/* Getters */


Config* DQNModelGetConfig(DQNModel *m) {
    return &(m->config);
}


/* Algorithmes */

/* ------------------------------- */

void valueIteration(DQNModel *m) {
    float epsilon = m->config.epsilon;
    float gamma = m->config.gamma;
    Env *env = m->userData;
    float delta = DBL_MAX;

    while (delta > epsilon) {
        delta = 0;

        for (int s = 0; s < EnvGetNS(env); ++s) {
            float oldValue = m->stateValues[s];
            float maxQ = -DBL_MAX;

            for (int a = 0; a < EnvGetNA(env); ++a) {
                float q = EnvGetR(env, s, a) + gamma * sum(
                    EnvGetTransitionArray(env, s, a), m->stateValues,EnvGetNS(env)
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

void policyEvaluation(DQNModel *m, int *policy) {
    float epsilon = m->config.epsilon;
    float gamma = m->config.gamma;
    Env *env = m->userData;
    float delta = DBL_MAX;

    while (delta > epsilon) {
        delta = 0;

        for (int s = 0; s < EnvGetNS(env); ++s) {
            float oldValue = m->stateValues[s];

            int a = policy[s];

            m->stateValues[s] = EnvGetR(env, s, a) + gamma * sum(
                EnvGetTransitionArray(env, s, a), m->stateValues, EnvGetNS(env)
            );

            delta = fmax(delta, fabs(m->stateValues[s] - oldValue));
        }
    }
}

bool policyImprovement(DQNModel *m, int *policy) {
    float gamma = m->config.gamma;
    Env *env = m->userData;

    bool isPolicyStable = true;

    for (int s = 0; s < EnvGetNS(env); ++s) {
        int oldAction = policy[s];

        float maxQ = -DBL_MAX;
        int bestAction = oldAction;

        for (int a = 0; a < EnvGetNA(env); ++a) {
            float q = EnvGetR(env, s, a) + gamma * sum(
                EnvGetTransitionArray(env, s, a), m->stateValues, EnvGetNS(env)
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

void policyIteration(DQNModel *m) {
    arrayRandom(m->policy, EnvGetNS(m->userData), EnvGetNA(m->userData));

    bool isPolicyStable = false;

    while (!isPolicyStable) {
        policyEvaluation(m, m->policy);
        isPolicyStable = policyImprovement(m, m->policy);
    }
}

/* ------------------------------- */

void QLearning(DQNModel *m) {
    float alpha = m->config.alpha;
    float gamma = m->config.gamma;
    Env *env = m->userData;

    for (int epoch = 0; epoch < m->config.epochs; ++epoch) {
        int state = 0;

        for (int step = 0; step < m->config.steps; ++step) {
            float r = (float)rand() / (float)RAND_MAX;
            int action;

            if (r < m->config.epsilon) {
                action =  rand() % EnvGetNA(env);
            } else {
                action = getBestAction(m, state);
            }

            int nextState = EnvGetTransitionState(env, state, action);
            float reward = EnvGetR(env, state, action);

            float nextValue = getBestNextQValue(m, nextState);

            m->QTable[getQIndex(m, state, action)] += alpha * (
                reward + (gamma * nextValue) - m->QTable[getQIndex(m, state, action)]
            );

            state = nextState;
        }
    }

    for (int s = 0; s < EnvGetNS(env); ++s) {
        m->policy[s] = getBestAction(m, s);
    }
}
