#ifndef __RL_CONFIG_H__
#define __RL_CONFIG_H__

#include <stdbool.h>
#include <stdio.h>

typedef struct s_rl_config Config;

/*
Structure contenant tous les hyperparamètres et paramètres
*/
struct s_rl_config {
    double gamma;
    double alpha;
    double epsilon;
    double epsilon_min;
    double epsilon_decay;
    int steps;
    int epochs;
};

Config defaultConfig(void);

void configSetGamma(Config *cfg, float v);

void configSetEpsilon(Config *cfg, float v);

void configSetAlpha(Config *cfg, float v);

void configSetSteps(Config *cfg, int v);

void configSetEpochs(Config *cfg, int v);

#endif
