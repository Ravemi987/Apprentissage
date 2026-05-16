#ifndef __RL_CONFIG_H__
#define __RL_CONFIG_H__

#include <stdbool.h>
#include <stdio.h>

typedef struct s_rl_config Config;

/*
Structure contenant tous les hyperparamètres et paramètres
*/
struct s_rl_config {
    double gamma;           // Discount factor : importance donnée aux récompenses futures (0 = aucune, 1 = importante)
    double epsilon;         // Taux d'exploration
    double epsilon_min;     // Taux minimum d'exploration (une fois que le réseau à bien appris)
    double epsilon_decay;   // Plus le réseau apprend, moins on a besoin de choisir une action au hasard
    int epochs;             // Nombre d'epochs max pour le Deep-Q-Learning (tous les combien on reset l'environnement)
    int max_steps;          // Nombre de steps par epoch
};

Config defaultConfig(void);

void configSetGamma(Config *cfg, float v);

void configSetEpsilon(Config *cfg, float v);

void configSetEpochs(Config *cfg, int v);

void configSetSteps(Config *cfg, int v);

void epsilonDecay(Config *cfg);

#endif
