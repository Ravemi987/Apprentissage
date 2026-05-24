#ifndef __RL_ENV_H__
#define __RL_ENV_H__

#include <stdbool.h>
#include <stdio.h>

#include "simulation.h"

#define GRID_SIZE 4
#define MAX_CLOSEST_OBSTACLES 3
#define NB_STATES (GRID_SIZE * GRID_SIZE + MAX_CLOSEST_OBSTACLES * 3 + 12)
#define FRAME_SKIP 5   // Frame Skipping pour laisser à la physique le temps de calculer les mouvements de l'IA


/*
 * Voici comment va être défini notre environnement :
 * Lorsqu'au veut apprendre au drone à aller quelque part, on ne doit pas seulement donner
 * au réseau de neurone en entrée sa position, vitesse, etc... Mais surtout sa position RELATIVE aux objets de l'environnment.
 * Problème : si on apprenait au drone d'aller d'un point A vers un point B, c'est simple, on donne target_x, target_y et target_z
 * Mais ici, on peut lancer le programme (via le main ou des arguments) avec un nombre arbitraire d'utilisateurs ou d'obstacles
 * Notre architecture de réseau de neurones devrait change à chaque fois !
 * Ici, on défini une grille de densité de 4 x 4 au sol, ce qui nous donne 16 états, auxquels on ajoute les états du drone (vitesse, angles, ...)
 * A chaque fois que l'on veut connaître le nombre d'utilisateurs, on regarde combien il y en a dans chaque case de la grille.
 * Du coup, peu importe le nombre d'utilisateurs, la grille a toujours la même taille, et le réseau reçoit une densité de population
 * Pour les objets, on implémente un système "simple" de style capteur LiDAR (les 3 objets les plus proches)
*/
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


#endif
