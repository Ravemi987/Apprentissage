#include "model.h"
#include "utils.h"

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdint.h>
#include <math.h>


/* Init */

static NeuralNetwork *initNetwork(int batchSize) {
    int layerSizes[] = {NB_STATES, 128, 64, NB_ACTION};
    int numLayers = sizeof(layerSizes) / sizeof(layerSizes[0]);
    return networkCreate(layerSizes, numLayers, "mean_squared_error", "relu", "linear", batchSize);
}


static ReplayBuffer *initReplayBuffer() {
    ReplayBuffer *b = malloc(sizeof(ReplayBuffer));
    b->capacity = MEMORY_SIZE;
    b->size = 0;
    b->head = 0;
    b->buffer = malloc(MEMORY_SIZE * sizeof(Transition));
    memset(b->buffer, 0, MEMORY_SIZE);
    return b;
}


DQNModel* DQNModelCreate(World *w, int update_freq, int batchSize) {
    DQNModel *m = malloc(sizeof(struct s_rl_model));

    m->batchSize = batchSize;
    m->q_network = initNetwork(batchSize);
    m->target_network = initNetwork(batchSize);
    m->memory = initReplayBuffer();
    m->env = initEnv(w);
    m->config = defaultConfig();
    m->train_step_count = 0;
    m->networks_update_freq = update_freq;
    return m;
};


void DQNModelDelete(DQNModel **m) {
    if ((*m) == NULL) return;

    networkDestroy(&(*m)->q_network);
    networkDestroy(&(*m)->target_network);
    free((*m)->memory->buffer);
    free((*m)->memory);
    free((*m)->env);
    free(*m);

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


/* Algorithm */

int predict(DQNModel *m, State state) {
    float r = (float)rand() / (float)RAND_MAX;
    int action;

    if (r < m->config.epsilon) {
        action =  rand() % NB_ACTION;
        epsilonDecay(&m->config);
    } else {
        double *q_values = nnForwardPropagation(m->q_network, state.features, m->batchSize);
        action = nnPredictClass(m, q_values);
    }

    return action;
}


void DeepQLearning(DQNModel *m) {
    float alpha = m->config.alpha;
    float gamma = m->config.gamma;
    State next_state;
    double reward;
    int is_terminal;
    Env *env = m->env;

    for (int epoch = 0; epoch < m->config.epochs; ++epoch) {
        resetEnv(env);

        for (int step = 0; step < m->config.steps; ++step) {
            int action = predict(m, env->current_state);
            envStep(env, &next_state, &reward, &is_terminal, action);

            Transition copy = {env->current_state, action, reward, next_state, is_terminal};
            saveTransition(m->memory, copy);

            env->current_state = next_state;

        }
    }
}
