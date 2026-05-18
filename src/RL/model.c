#include "model.h"
#include "utils.h"

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdint.h>
#include <math.h>


/*
 * Ici on créé un réseau de neurones avec en entrée les données qui définissent un état (vitesse, position, ...),
 * deux hidden layers (normalement suffisant: 128 * 64 ou 64 * 64 ou 128 * 128 par exemple en fonction des perfs)
 * et le vecteur de valeurs des actions. La valeur la plus élevée donne l'action à prendre
 * On utilise la MSE.
 * Pour la navigation, ReLU fait le taffe comme fonction d'activation pour les couches cachées car c'est rapide
 * et ça évite le problème du vanishing gradient avec l'équation de Bellman.
 * En sortie, on a absolument besoin de Linear, car on a besoin des valeurs exactes prédites pour l'état suivant (position, ...)
 * (Softmax et Sigmoid donnent des probas donc poubelle).
 * A la limite, tweaker les couches internes et la batchsize.
*/
static NeuralNetwork *initNetwork(int batchSize) {
    int layerSizes[] = {NB_STATES, 256, 256, NB_ACTION};
    int numLayers = sizeof(layerSizes) / sizeof(layerSizes[0]);
    return networkCreate(layerSizes, numLayers, "mean_squared_error", "silu", "linear", batchSize);
}


static ReplayBuffer *initReplayBuffer() {
    ReplayBuffer *b = malloc(sizeof(ReplayBuffer));
    b->capacity = MEMORY_SIZE;
    b->size = 0;
    b->head = 0;
    b->buffer = calloc(MEMORY_SIZE, sizeof(Transition));
    return b;
}


/* 
 * Initialise un modèle DQN avec tout ce qu'il faut
 * On passe une config par defaut, on ne choisit que la fréquence de synchronisation des DNN,
 * et la taille d'un batch
*/
DQNModel* DQNModelCreate(World *w, int update_freq, int batchSize, double learningRate, double decay) {
    DQNModel *m = malloc(sizeof(struct s_rl_model));

    m->batchSize = batchSize;
    m->q_network = initNetwork(batchSize);
    m->target_network = initNetwork(batchSize);
    m->memory = initReplayBuffer();
    m->config = defaultConfig();
    m->env = initEnv(w, m->config.max_steps);
    m->step_count = 0;
    m->learningRate = learningRate;
    m->decay = decay;
    m->networks_update_freq = update_freq;

    m->batch_inputs = malloc(m->batchSize * NB_STATES * sizeof(double));
    m->batch_next_inputs = malloc(m->batchSize * NB_STATES * sizeof(double));
    m->batch_expected_outputs = malloc(m->batchSize * NB_ACTION * sizeof(double));

    return m;
};


void DQNModelDelete(DQNModel **m) {
    if ((*m) == NULL) return;

    networkDestroy(&(*m)->q_network);
    networkDestroy(&(*m)->target_network);
    free((*m)->memory->buffer);
    free((*m)->memory);
    free((*m)->env);
    free((*m)->batch_inputs);
    free((*m)->batch_next_inputs);
    free((*m)->batch_expected_outputs);
    free(*m);

    (*m) = NULL;
}


void DQNModelSetConfig(DQNModel *m, Config cfg) {
    m->config = cfg;
}


Config* DQNModelGetConfig(DQNModel *m) {
    return &(m->config);
}


void DQNModelSetPath(DQNModel *m, char *path) {
    m->path = path;
}


/*
 * Fonction appelée dans l'algo du Deep-Q-Learning et qui fait deux choses:
 * - Décide si on explore (valeur aléatoire dépendant d'epsilon) ou si on exploite (prédiction avec le réseau)
 * - Dans ce cas, on fait une forward pass et un argmax pour récupérer la meilleure action
 */
int predict(DQNModel *m, double *state) {
    float r = (float)rand() / (float)RAND_MAX;
    int action;

    if (r < m->config.epsilon) {
        action =  rand() % NB_ACTION;
    } else {
        double *q_values = nnForwardPropagation(m->q_network, state, 1);
        action = arrayMaxIndex(q_values, NB_ACTION);
    }

    return action;
}


void copyTransition(Transition *copy, double *current_state, int action, double reward, double *next_state, int is_terminal) {
    // On copie les données une par une
    memcpy((*copy).state, current_state, NB_STATES * sizeof(double));
    memcpy((*copy).next_state, next_state, NB_STATES * sizeof(double));

    (*copy).action = action;
    (*copy).reward = reward;
    (*copy).next_state_terminal = is_terminal;
}


void saveTransition(ReplayBuffer *b, Transition t) {
    b->buffer[b->head] = t; // Copie des données
    b->head = (b->head + 1) % b->capacity; // Comportement circulaire
    if (b->size < b->capacity) {
        b->size++;
    }
}


/* Fonction qui retourne un batch aléatoire du ReplayBuffer (voir model.h) */
void getRandomBatch(ReplayBuffer *r, Transition *batch, int batchSize) {
    if (r->size < batchSize) return;

    for (int i = 0; i < r->size; i++) {
        r->buffer[i].flag = 0;
    }

    int count = 0;

    while (count < batchSize) {
        int randomIndex = rand() % r->size;

        if (r->buffer[randomIndex].flag == 0) {
            batch[count] = r->buffer[randomIndex];
            r->buffer[randomIndex].flag = 1;
            count++;
        }
    }
}


/*
 * Fonction qui entraine le q_network, voir la fonction DeepQLearning.
 * L'idée est de construire artificiellement les expected_outputs pour forcer le réseau de neurones 
 * à n'apprendre que de l'action qu'il a réellement vécue, sans toucher au reste (les autres poids).
*/
void updateNetwork(DQNModel *m) {
    Transition batch[m->batchSize];
    getRandomBatch(m->memory, batch, m->batchSize); // On commence par récupérer un batch de données passées

    // On prend un batch complet du ReplayBuffer pour entraîner le réseau
    #pragma omp parallel for
    for (int i = 0; i < m->batchSize; ++i) {
        // On sauvegarde les inputs !
        memcpy(&m->batch_inputs[i * NB_STATES], batch[i].state, NB_STATES * sizeof(double));
        memcpy(&m->batch_next_inputs[i * NB_STATES], batch[i].next_state, NB_STATES * sizeof(double));
    }

    // Première prédiction nous donne l'évaluation COURANTE (q_network) des valeurs des actions dans l'ancien état S
    double *all_current_q = nnForwardPropagation(m->q_network, m->batch_inputs, m->batchSize);
    // Deuxième prédiction sur S' (l'ancien état suivant) avec le Target Network pour inclure les estimations futures
    double *all_next_q =  nnForwardPropagation(m->target_network, m->batch_next_inputs, m->batchSize);

    // On construit expectedOutput
    #pragma omp parallel for
    for (int i = 0; i < m->batchSize; ++i) {
        double *current_q = &all_current_q[i * NB_ACTION]; // On récupère l'estimation courante

        // On copie ces valeurs dans notre tableau d'expected_output. De ce fait, les actions non choisies n'impacteront pas les poids
        memcpy(&m->batch_expected_outputs[i * NB_ACTION], current_q, NB_ACTION * sizeof(double));

        // Estimation futur (c'est la récompense immédiate obtenue en ayant choisi l'action A)
        double target_value = batch[i].reward;

        // S'il y avait un état suivant, on récupère la meilleure estimation (Equation de Bellman)
        if (batch[i].next_state_terminal == 0) {
            double *next_q = &all_next_q[i * NB_ACTION];
            double max_q = arrayMax(next_q, NB_ACTION);
            target_value += m->config.gamma * max_q;
        }

        // A ce stade, on injecte dans expected_outputs pour remplacer la valeur de l'action choisie par la meilleure (dans ce batch)
        m->batch_expected_outputs[i * NB_ACTION + batch[i].action] = target_value;
    }

    // On entraîne maintenant le réseau sur le batch
    networkTrain(m->q_network, m->batch_inputs, m->batch_expected_outputs, m->batchSize, m->learningRate, 1, m->batchSize, m->decay);
}


/* 
 * Algorithme d'entraînement.
 * 
*/
void DeepQLearning(DQNModel *m) {
    double next_state[NB_STATES];
    double reward;
    int is_terminal;
    Env *env = m->env;

    // On fait un certain nombre d'epochs (entraînement complet) pour valider la généralisation du réseau
    for (int epoch = 0; epoch < m->config.epochs; ++epoch) {
        resetEnv(env, epoch);  // Au début de chaque epochs, il faut repartir de l'état initial (comme le Q-Learning classique)
        double total_epoch_reward = 0.0;

        for (m->step_count = 0; m->step_count < env->max_steps; ++(m->step_count)) {
            // On choisit l'action à prendre (action réelle du drone)
            int action = predict(m, env->current_state);

            // Transition
            envStep(env, next_state, &reward, &is_terminal, action);
            total_epoch_reward += reward;

            // On sauvegarde : état de départ, action prise, récompense obtenue, état d'arrivée
            Transition copy;
            copyTransition(&copy, env->current_state, action, reward, next_state, is_terminal);
            saveTransition(m->memory, copy);

            // On passe au prochain état
            memcpy(env->current_state, next_state, NB_STATES * sizeof(double));

            // On met à jour le réseau
            if (m->memory->size > m->batchSize) {
                updateNetwork(m);
            }

            // On met à jour le clone (target_network)
            if (m->step_count % m->networks_update_freq == 0) {
                networkCopyWeights(m->target_network, m->q_network);
            }

            if (is_terminal) break;
        }

        epsilonDecay(&m->config);

        printf("Epoch %4d/%d | Steps: %4d | Total Reward: %7.2f | Epsilon: %.3f\n", 
               epoch + 1, m->config.epochs, m->step_count, total_epoch_reward, m->config.epsilon);

        if ((epoch + 1) % 50 == 0) {
            printf(">>> Sauvegarde automatique (Epoch %d) ! <<<\n", epoch + 1);
            networkSave(m->q_network, m->path);
        }
    }
}
