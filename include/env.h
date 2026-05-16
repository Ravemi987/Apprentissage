#ifndef __RL_ENV_H__
#define __RL_ENV_H__

#include <stdbool.h>
#include <stdio.h>

#include "simulation.h"


typedef struct s_rl_env {
    World *physical_world;              // Monde physique
    
    double current_state[NB_STATES];    // Etat courant, important pour pouvoir avancer dans l'apprentissage (passer d'état en état)
    int is_terminal;                    // Savoir si l'état est terminal : l'agent atteint son objectif (ex: max_step) ou a échoué (crash ,...)
    double current_reward;              // Récompense courante (dernière reçue)

    double target_x;          // Coordonnées du point qu'on veut atteindre
    double target_y;
    double target_z;

    int max_steps;            // Nombre d'étapes maximum d'étapes que le drone peut faire (batterie max)
} Env;


void envStep(Env *env, double *next_state, double *reward, int *is_terminal, int action_idx, int step_count);

int isTerminalState(Env *env, int step_count);

double getReward(Env *env);

double computeDistanceToTarget(Env *env);

void resetEnv(Env *env);

Env *initEnv(World *w, int max_steps, double *target);


#endif
