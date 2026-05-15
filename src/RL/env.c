#include "env.h"
#include <stdlib.h>

#define TARGET_RADIUS 1.0   // Distance pour considérer la cible atteinte
#define SAFETY_RADIUS 2.0   // Distance de sécurité avec les utilisateurs


double computeDistanceToTarget(Env *env) {
    World *w = env->physical_world;
    Drone *d = w->drone;

    double dist = sqrt(pow(d->x - env->target_x, 2) + 
                       pow(d->y - env->target_y, 2) + 
                       pow(d->z - env->target_z, 2));

    return dist;
}


/* Détermine si oui ou non on a terminé */
int isTerminalState(Env *env, int step_count) {
    World *w = env->physical_world;

    // Crash
    if (isDroneCrashed(w)) {
        return 1;
    }

    // Cible atteinte
    if (computeDistanceToTarget(env) < TARGET_RADIUS) {
        return 1;
    }

    // Timeout
    if (step_count >= env->max_steps) {
        return 1;
    }

    return 0;
}


/* Détermine la valeur de la récompense : temps, distance, stabilité, obstacles, crash et objectif atteint */
double getReward(Env *env) {
    World *w = env->physical_world;
    Drone *d = w->drone;
    double reward = 0.0;

    // Pénalité de temps
    reward -= 0.1;

    // Pénalité de distance
    double dist = computeDistanceToTarget(env);
    reward -= dist * 0.5;

    // Pénalité de non stabilité
    reward -= (fabs(d->phi) + fabs(d->theta)) * 0.2; // Roulis et Tangage
    reward -= (fabs(d->p) + fabs(d->q) + fabs(d->r)) * 0.05; // Vitesses angulaires

    // Pénalité de non évitement d'obstacles / utilisateurs
    for (int i = 0; i < w->numUsers; i++) {
        double dist_user = sqrt(pow(d->x - w->users[i].x, 2) + 
                                pow(d->y - w->users[i].y, 2) + 
                                pow(d->z - w->users[i].z, 2));
        if (dist_user < SAFETY_RADIUS) {
            reward -= 5.0;
        }
    }

    // Pour les récompenses terminales, il faut utiliser les mêmes règles que TerminalState
    if (isDroneCrashed(w)) {
        reward -= 100.0;
    } else if (dist < TARGET_RADIUS) {
        reward += 100.0;
    }

    return reward;
}


/*
 * Cette fonction définit de quelles informations l'IA a besoin pour savoir ce que doit faire
 * le drone à chaque instant.
 * On ne donne pas la position absolue du drone, mais ça distance par rapport à la cible.
 * On a aussi besoin de la vitesse, des angles et de la vitesse angulaire.
*/
void getStateVector(Env *env, double *state_out) {
    Drone *d = env->physical_world->drone;

    // Position relative normalisée entre -1 et 1
    state_out[0] = (env->target_x - d->x) / env->physical_world->width;
    state_out[1] = (env->target_y - d->y) / env->physical_world->height;
    state_out[2] = (env->target_z - d->z) / env->physical_world->depth;

    // Vitesses linéaires normalisées entre -1 et 1
    state_out[3] = d->x_dot / MAX_VELOCITY;
    state_out[4] = d->y_dot / MAX_VELOCITY;
    state_out[5] = d->z_dot / MAX_VELOCITY;

    // Angles (déjà normalisés entre -1 et 1 car bornés à [-pi, pi])
    state_out[6] = d->phi;
    state_out[7] = d->theta;
    state_out[8] = d->psi;

    // Vitesses angulaires normalisées entre -1 et 1
    state_out[9] = d->p / MAX_ROT;
    state_out[10] = d->q / MAX_ROT;
    state_out[11] = d->r / MAX_ROT;
}


void envStep(Env *env, double *next_state, double *reward, int *is_terminal, int action_idx, int step_count) {
    // L'action en paramètre définie quelle commande envoyer au drone (la commande est d'abord traitée par le controller)
    handleCommand(env->physical_world->drone, action_idx);

    // On fait tous les calculs physiques
    physicsStep(env->physical_world, DT);

    // On est dans un nouvelle état (nouvelle position, notre vitesse a augmenté, etc...)
    getStateVector(env, next_state);

    // On reçoit une récompense et on détermine si l'état est terminal
    *reward = getReward(env);
    *is_terminal = isTerminalState(env, step_count);
}


void resetEnv(Env *env) {
    World *w = env->physical_world;

    *(w->drone) = createDrone(100.0, 50.0, 20.0);
    env->current_reward = 0.0;
    env->is_terminal = 0;

    getStateVector(env, env->current_state);
}


Env *initEnv(World *w, double *target) {
    Env *env = malloc(sizeof(Env));

    env->physical_world = w;
    env->current_reward = 0.0;
    env->is_terminal = 0;
    env->target_x = target[0];
    env->target_y = target[1];
    env->target_z = target[2];
    env->max_steps = 1000;

    getStateVector(env, env->current_state);

    return env;
}
