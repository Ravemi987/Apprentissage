#ifndef __RL_ENV_H__
#define __RL_ENV_H__

#include <stdbool.h>
#include <stdio.h>
#include "state.h"

#include "simulation.h"


typedef struct s_rl_env {
    World *physical_world;  // Monde physique
    
    State current_state;    // Etat courant
    int is_terminal;        // Pour savoir si l'état est terminal : l'agent atteint son objectif ou a échoué
    double current_reward;

    double target_x;
    double target_y;
    double target_z;
    int step_count;         // Calcul du nombre d'étapes de la simulations physiques
    int max_steps;          // Nombre d'étapes maximum de simulation de la physique
} Env;


void envStep(Env *env, State *next_state, double *reward, int *is_terminal, int action_idx);

void resetEnv(Env *env);

Env *initEnv(World *w);

#endif
