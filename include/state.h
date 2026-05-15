#ifndef __RL_STATE_H__
#define __RL_STATE_H__

#include <stdbool.h>
#include <stdio.h>
#include "simulation.h"


typedef struct s_rl_state {
    float features[NB_STATES];   // Les données prises en compte : position, vitesse linéaire, vitesse angulaire, ...
} State;


void getStateVector(World *w, State *s, double target_x, double target_y, double target_z);

#endif
