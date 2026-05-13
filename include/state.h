#ifndef __RL_STATE_H__
#define __RL_STATE_H__

#include <stdbool.h>
#include <stdio.h>
#include "simulation.h"

#define MAX_FEATURES 30

typedef struct s_rl_state {
    float features[MAX_FEATURES];   // Les données prises en compte : position, vitesse linéaire, vitesse angulaire, ...
    int nb_features;
} State;

double normalizeState(State* s, World *world);

#endif
