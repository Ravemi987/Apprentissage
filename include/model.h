#ifndef __RL_MODEL_H__
#define __RL_MODEL_H__

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "config.h"
#include "network.h"
#include "env.h"

#define MEMORY_SIZE 100000


/*
 * Dans le Q-Learning classique, on choisi une action, on observe le nouvel état,
 * et on met à jour la table. Ici si on fait ça, on va avoir un problème de corrélation des données.
 * Si à l'instant t, le drone est en (x = 10, y = 10), à l'instant t + 0.01 (DT), il est en (x = 10.1, y = 10).
 * Ces deux états sont quasi identiques. Si on entraîne le réseau dessus, il va overfitter sur cette trajectoire et 
 * oublier complètement comment voler. On doit donc sauvegarder les étapes:
 * Etat S, Action A, Récompense R et Etat suivant S' dans un buffer. Au lieu de prendre la dernière action effectuée,
 * on tire aléatooirement un mini-batch (ex: 64) dans ce buffer pour que le réseau apprenne sur un mélange de situations passées
 * et présentes pour casser la corrélation.
 * 
*/

typedef struct {
    double state[NB_STATES];      // Etat avant l'action
    EngineAction action;          // Action qu'on prend
    double reward;                // Recompense qu'on obtient
    double next_state[NB_STATES]; // Prochaine état dans lequel on se trouve
    int next_state_terminal;      // Est-ce que le prochain état est terminal (voir Env) ?
    int flag;                     // Champ utilitaire, ici utilisé pour choisir un random batch
} Transition;


/*
 * Le ReplayBuffer est la structure qui permet de stocker toutes les transitions (voir ci-dessus)
*/

typedef struct {
    Transition *buffer; // Tableau alloué dynamiquement pour mémoriser les transitions
    int capacity;       // Capacité max du buffer
    int size;           // Taille actuelle du buffer
    int head;           // Index d'insertion (buffer dynamique)
} ReplayBuffer;


/*
 * Notre modèle de Deep-Q-Network contient deux réseaux de neurones.
 * En effet, l'équation de Bellman nous dit: Q(S, A) <- Q(S, A) + alpha * [R + gamma * max Q(S', a') - Q(S, A)]
 * avec alpha = taux d'apprentissage et gamma = discount factor.
 * La récompense qu'on veut atteindre est y = R + gamma * max Q(s', a') (la différence donne l'erreur)
 * Mais si on utilise un seul réseau de neurones, on calcule l'erreur et l'estimation avec les mêmes poids !
 * Donc quand on met à jour les poids, après le calcul de l'erreur et la rétropropagation, on change aussi l'estimation
 * pour l'itération suivante et le réseau n'apprend pas.
 * 
 * Solution DeepMind (2015)
 * On a le q_network : qu'on entraîne à chaque étape
 * le target_network : une copie "freez" qui sert juste à calculer l'erreur
 * Toutes les networks_update_freq, on copie les poids du q_network vers le target_network (la cible à un peu bougé)
*/

typedef struct s_rl_model {
    NeuralNetwork *q_network;       // Réseau principal (entraine Q(s, a))
    NeuralNetwork *target_network;  // Réseau cible (calcule max Q(s', a'))
    Env *env;                       // Environnement
    ReplayBuffer *memory;           // Mémoire pour l'Experience Replay
    Config config;                  // Config avec hyperparamètres

    int batchSize;                  // Taille des batchs pour les réseaux de neurones
    double learningRate;            // Pour l'entraînement du réseau de neurone
    double decay;                   // Decay pour le learning rate du réseau de neurone
    int networks_update_freq;       // Fréquence de synchronisation des deux réseaux (ex: 1000 steps)
    int step_count;                 // Compteur pour savoir quand synchroniser
    char *path;                     // Chemin de sauvegarde

    // Buffers pour éviter la réallocation lors de l'entraînement
    double *batch_inputs;
    double *batch_next_inputs;
    double *batch_expected_outputs;
} DQNModel;


void DeepQLearning(DQNModel *m);

DQNModel* DQNModelCreate(World *w, int update_freq, int batchSize, double learningRate, double decay);

void DQNModelDelete(DQNModel **m);

void DQNModelSetConfig(DQNModel *m, Config cfg);

Config* DQNModelGetConfig(DQNModel *m);

void DQNModelSetPath(DQNModel *m, char *path);

int predict(DQNModel *m, double *state);

#endif
