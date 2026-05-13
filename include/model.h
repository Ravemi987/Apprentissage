#ifndef __RL_MODEL_H__
#define __RL_MODEL_H__

#include <stdbool.h>
#include <stdio.h>

#include "config.h"
#include "network.h"
#include "state.h"

typedef struct {
    State state;
    EngineAction action;
    double reward;
    State next_state;
    int next_state_terminal;
} Transition;


typedef struct {
    Transition *buffer; // Tableau alloué dynamiquement mémoriser les transitions
    int capacity;       // Capacité max
    int size;           // Taille actuelle
    int head;           // Index d'insertion (buffer dynamique)
} ReplayBuffer;


typedef struct s_rl_model {
    NeuralNetwork *q_network;       // Réseau principal (entraine Q(s, a))
    NeuralNetwork *target_network;  // Réseau cible (calcule max Q(s', a'))

    ReplayBuffer *memory;           // Mémoire pour l'Experience Replay
    Config config;                  // Config avec hyperparamètres

    int target_update_freq;         // Fréquence de synchronisation des deux réseaux (ex: 1000 steps)
    int train_step_count;           // Compteur pour savoir quand synchroniser
} DQNModel;



#endif
