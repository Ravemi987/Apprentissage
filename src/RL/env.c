#include "env.h"
#include <stdlib.h>
#include <generation.h>
#include "utils.h"


/* Calcule la pénalité liée aux obstacles (Arbres, bâtiments) */
static double getObstaclePenalty(World *w, Drone *d) {
    double penalty = 0.0;
    for (int i = 0; i < w->numObstacles; i++) {
        Obstacle3D obs = w->obstacles[i];
        
        // Si le drone vole à la hauteur de l'obstacle
        if (d->z >= obs.z && d->z <= (obs.z + obs.height)) {
            double dist_h = sqrt(pow(d->x - obs.x, 2) + pow(d->y - obs.y, 2));
            double safety_zone = obs.radius + SAFETY_RADIUS; 
            
            if (dist_h < safety_zone) {
                double penetration = (safety_zone - dist_h) / SAFETY_RADIUS;
                penalty -= clamp(penetration, 0.0, 1.0) * 2.0; 
            }
        }
    }
    return penalty;
}


/* Trouve l'utilisateur le plus proche et le définit comme cible */
void updateTargetToClosestUser(World *w) {
    Drone *d = w->drone;
    if (w->numUsers <= 0) return;

    double min_dist_sq = 999999.0;
    int closest_idx = -1;

    for (int i = 0; i < w->numUsers; i++) {
        // Distance euclidienne au carré (plus rapide car pas de sqrt)
        double dist_sq = pow(d->x - w->users[i].x, 2) + pow(d->y - w->users[i].y, 2);
        
        if (dist_sq < min_dist_sq && !w->users[i].is_reached) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    if (closest_idx != -1) {
        d->target_x = w->users[closest_idx].x;
        d->target_y = w->users[closest_idx].y;
        d->target_z = w->users[closest_idx].z + 2.5; 
    }
}


void checkTerminal(double *final_reward, double *reward, int *is_terminal, int crashed, World *w) {
    Drone *d = w->drone;

    double dist_to_target = sqrt(pow(d->target_x - d->x, 2) + 
                                 pow(d->target_y - d->y, 2) + 
                                 pow(d->target_z - d->z, 2));
                                 
    int reached_target = (dist_to_target <= 1.5);

    if (reached_target) {
        *final_reward += 100.0;
        *is_terminal = 1;
    } else if (crashed) { 
        *final_reward -= 20.0;
    }

    *reward = *final_reward;
    *is_terminal = crashed || reached_target;
}


double getReward(Env *env) {
    World *w = env->physical_world;
    Drone *d = w->drone;
    
    // Calcul de la distance euclidienne tridimensionnelle vers la cible stockée dans le drone
    double dist = sqrt(pow(d->target_x - d->x, 2) + pow(d->target_y - d->y, 2) + pow(d->target_z - d->z, 2));
    
    // Normalisation de la distance
    double max_map_dist = sqrt(pow(w->width, 2) + pow(w->height, 2) + pow(w->depth, 2));
    double norm_dist = clamp(dist / max_map_dist, 0.0, 1.0);
    
    // Base reward : donne une valeur entre 0.1 (très loin) et 5.1 (exactement dessus)
    double base_reward = 0.1 + pow(1.0 - norm_dist, 2) * 5.0; 
    
    // Multiplicateur de sécurité (murs physiques et limites de vol)
    double safety_multiplier = 1.0;
    double margin = 10.0;
    
    safety_multiplier *= clamp(d->x / margin, 0.2, 1.0);
    safety_multiplier *= clamp((w->width - d->x) / margin, 0.2, 1.0);
    safety_multiplier *= clamp(d->y / margin, 0.2, 1.0);
    safety_multiplier *= clamp((w->height - d->y) / margin, 0.2, 1.0);
    
    // Sécurité d'altitude
    safety_multiplier *= clamp((d->z - 2.0) / 3.0, 0.2, 1.0);
    safety_multiplier *= clamp((35.0 - d->z) / 10.0, 0.2, 1.0);
    
    // Intégration des pénalités d'obstacles
    double obs_pen = fabs(getObstaclePenalty(w, d)); 
    
    // On convertit les pénalités cumulées en un multiplicateur d'environnement (de 0.1 à 1.0)
    double env_multiplier = clamp(1.0 - obs_pen, 0.1, 1.0);
    safety_multiplier *= env_multiplier;
    
    return base_reward * safety_multiplier;
}

/*
 * Cette fonction définit de quelles informations l'IA a besoin pour savoir ce que doit faire
 * le drone à chaque instant.  Elle renvoit donc un vecteur de valeurs qui définissent notre monde et qui
 * set d'entrée au réseau de neurone. Cela inclut les positions relatives aux utilisateurs, mais aussi aux obstacles.
 * Voir les commentaires et aussi le fichier env.h pour plus d'infos.
*/
void getStateVector(Env *env, double *state_out) {
    World *w = env->physical_world;
    Drone *d = w->drone;

    state_out[0] = clamp((d->target_x - d->x) / w->width, -1.0, 1.0);
    state_out[1] = clamp((d->target_y - d->y) / w->height, -1.0, 1.0);
    state_out[2] = clamp((d->target_z - d->z) / w->depth, -1.0, 1.0);

    // Deuxième étape : capture des obstacles (LiDAR)
    captureObstacles(env, state_out);

    //Troisième étape : variables physiques du drone (comme avec une target A -> B)
    int drone_offset =  3 + (MAX_CLOSEST_OBSTACLES * 3);

    state_out[drone_offset + 0] = clamp(d->z / SAFETY_RADIUS, 0.0, 1.0);
    state_out[drone_offset + 1] = clamp(d->z / w->depth, 0.0, 1.0);
    state_out[drone_offset + 2] = clamp(d->x_dot / MAX_VELOCITY, -1.0, 1.0);
    state_out[drone_offset + 3] = clamp(d->y_dot / MAX_VELOCITY, -1.0, 1.0);
    state_out[drone_offset + 4] = clamp(d->z_dot / MAX_VELOCITY, -1.0, 1.0);
    state_out[drone_offset + 5] = clamp(d->phi / ANGLE_LIMIT, -1.0, 1.0);
    state_out[drone_offset + 6] = clamp(d->theta / ANGLE_LIMIT, -1.0, 1.0);
    state_out[drone_offset + 7] = clamp(d->psi / MATH_PI, -1.0, 1.0);
    state_out[drone_offset + 8] = clamp(d->p / MAX_ROT, -1.0, 1.0);
    state_out[drone_offset + 9] = clamp(d->q / MAX_ROT, -1.0, 1.0);
    state_out[drone_offset + 10] = clamp(d->r / MAX_ROT, -1.0, 1.0);
}


/* Fais un pas pour calculer le prochain état physique */
void envStep(Env *env, double *next_state, double *reward, int *is_terminal, int action_idx) {
    World *w = env->physical_world;
    double accumulated_reward = 0.0;
    int crashed = 0;

    handleCommand(w->drone, action_idx);

    // On laisse le drone exécuter l'action choisie pendant FRAME_SKIP itérations physiques
    for (int i = 0; i < FRAME_SKIP; i++) {
        physicsStep(w, DT);
        crashed = isDroneCrashed(w);
        accumulated_reward += getReward(env);
        if (crashed) break;
    }

    getStateVector(env, next_state);
    double final_reward = accumulated_reward / FRAME_SKIP; 


    checkTerminal(&final_reward, reward, is_terminal, crashed, w);
}


/* Initialise l'environnement en capturant la configuration dynamique du monde */
Env *initEnv(World *w, int max_steps) {
    Env *env = malloc(sizeof(Env));

    env->physical_world = w;
    env->current_reward = 0.0;
    env->is_terminal = 0;
    env->max_steps = max_steps;

    // Sauvegarde de la position de départ du drone
    env->spawn_drone_x = w->drone->x;
    env->spawn_drone_y = w->drone->y;
    env->spawn_drone_z = w->drone->z;

    // Allocation des tableaux de sauvegarde pour les utilisateurs
    env->spawn_users_x = malloc(w->numUsers * sizeof(double));
    env->spawn_users_y = malloc(w->numUsers * sizeof(double));
    for (int i = 0; i < w->numUsers; i++) {
        env->spawn_users_x[i] = w->users[i].x;
        env->spawn_users_y[i] = w->users[i].y;
    }

    // Allocation des tableaux de sauvegarde pour les obstacles
    env->spawn_obs_x = malloc(w->numObstacles * sizeof(double));
    env->spawn_obs_y = malloc(w->numObstacles * sizeof(double));
    for (int i = 0; i < w->numObstacles; i++) {
        env->spawn_obs_x[i] = w->obstacles[i].x;
        env->spawn_obs_y[i] = w->obstacles[i].y;
    }

    getStateVector(env, env->current_state);

    return env;
}


/* Réinitialise l'environnement de manière dynamique et sécurisée */
void resetEnv(Env *env, int current_epoch) {
    World *w = env->physical_world;

    if (current_epoch < TUTORIAL_EPOCHS) {
        *(w->drone) = createDrone(env->spawn_drone_x, env->spawn_drone_y, env->spawn_drone_z);
        
        // Réinitialisation de l'état (et des cibles) des utilisateurs
        for (int i = 0; i < w->numUsers; i++) {
            w->users[i].x = env->spawn_users_x[i];
            w->users[i].y = env->spawn_users_y[i];
            w->users[i].is_reached = 0; 
        }
    } 
    else {
        if (current_epoch % 30 == 0) {
            majWorld(w, TOTAL_RAND);
        }

        // On reset l'état is_reached même si le monde n'a pas été maj (les 29 autres epochs)
        for (int i = 0; i < w->numUsers; i++) {
            w->users[i].is_reached = 0;
        }

        // On cherche des coordonnées aléatoires pour le drone
        double spawn_x, spawn_y, spawn_z;
        getSafeDroneSpawn(w, &spawn_x, &spawn_y, &spawn_z);
        *(w->drone) = createDrone(spawn_x, spawn_y, spawn_z);
    }

    updateTargetToClosestUser(w);
    
    env->current_reward = 0.0;
    env->is_terminal = 0;

    getStateVector(env, env->current_state);
}