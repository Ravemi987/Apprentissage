#ifndef __RL_ENV_H__
#define __RL_ENV_H__

#include <stdbool.h>
#include <stdio.h>

#include "simulation.h"

#define MAX_CLOSEST_OBSTACLES 3
#define NB_STATES (3 + MAX_CLOSEST_OBSTACLES * 3 + 11)
#define FRAME_SKIP 10   // Frame Skipping pour laisser à la physique le temps de calculer les mouvements de l'IA
#define TUTORIAL_EPOCHS 200


typedef struct s_rl_env {
    World *physical_world;              // Monde physique
    
    double current_state[NB_STATES];    // Etat courant, important pour pouvoir avancer dans l'apprentissage (passer d'état en état)
    int is_terminal;                    // Savoir si l'état est terminal : l'agent atteint son objectif (ex: max_step) ou a échoué (crash ,...)
    double current_reward;              // Récompense courante (dernière reçue)

    int max_steps;                      // Nombre d'étapes maximum d'étapes que le drone peut faire (batterie max)

    double *spawn_users_x;
    double *spawn_users_y;
    double *spawn_obs_x;
    double *spawn_obs_y;
    
    double spawn_drone_x;
    double spawn_drone_y;
    double spawn_drone_z;
} Env;


void envStep(Env *env, double *next_state, double *reward, int *is_terminal, int action_idx);

double getReward(Env *env);

void resetEnv(Env *env, int current_epoch);

Env *initEnv(World *w, int max_steps);

void getStateVector(Env *env, double *state_out);

#endif
