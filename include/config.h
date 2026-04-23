#ifndef __RL_CONFIG_H__
#define __RL_CONFIG_H__

#include <stdbool.h>
#include <stdio.h>

typedef struct s_rl_config RLConfig;

/*
Structure contenant tous les hyperparamètres et paramètres
*/
struct s_rl_config {
    float gamma;
    float epsilon;
    float alpha;
    int steps;
    int epochs;
};

RLConfig RLDefaultConfig(void);

void RLConfigSetGamma(RLConfig *cfg, float v);

void RLConfigSetEpsilon(RLConfig *cfg, float v);

void RLConfigSetAlpha(RLConfig *cfg, float v);

void RLConfigSetSteps(RLConfig *cfg, int v);

void RLConfigSetEpochs(RLConfig *cfg, int v);

#endif
