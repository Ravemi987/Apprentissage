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


/* Calcule la pénalité de sécurité humaine (Ne pas voler trop bas au-dessus des gens) */
static double getHumanProximityPenalty(World *w, Drone *d) {
    double penalty = 0.0;
    
    if (d->z >= 0.0 && d->z <= SAFETY_RADIUS) {
        for (int i = 0; i < w->numUsers; i++) {
            User u = w->users[i];
            double dist_h = sqrt(pow(d->x - u.x, 2) + pow(d->y - u.y, 2));
            
            if (dist_h < SAFETY_RADIUS) {
                double penetration = (SAFETY_RADIUS - dist_h) / SAFETY_RADIUS;
                penalty -= clamp(penetration, 0.0, 1.0) * 2.0;
            }
        }
    }
    return penalty;
}


/* Calcule à la fois le pire signal, le signal moyen et le nombre d'utilisateurs connectés */
static double getSignalMetrics(World *w, Drone *d, double *out_average_norm) {
    if (w->numUsers <= 0) {
        *out_average_norm = 0.0;
        return 0.0;
    }

    double min_norm = 1.0; // On commence au maximum possible pour trouver le pire
    double total_norm = 0.0;

    for (int i = 0; i < w->numUsers; i++) {
        double rssi = computeRSSI(d, &w->users[i]);
        if (isnan(rssi) || isinf(rssi)) rssi = -100.0;

        // Normalisation
        double norm = clamp((rssi + 100.0) / 70.0, 0.0, 1.0);
        
        // Accumulation pour la moyenne
        total_norm += norm;

        // Sauvegarde du pire signal (Min)
        if (norm < min_norm) {
            min_norm = norm;
        }
    }

    // On extrait la moyenne via le pointeur de sortie
    *out_average_norm = total_norm / w->numUsers;

    // On retourne le min de manière classique
    return min_norm;
}


// double getReward(Env *env) {
//     World *w = env->physical_world;
//     Drone *d = w->drone;
    
//     double average_signal_norm = 0.0;
//     double min_signal_norm = getSignalMetrics(w, d, &average_signal_norm);
    
//     double signal_score = (min_signal_norm * 0.5) + (average_signal_norm * 0.5);
    
//     double reward = (0.1 + (pow(signal_score, 2) * 5.0));
    
//     double safety_multiplier = 1.0;
    
//     double margin = 10.0;
//     safety_multiplier *= clamp(d->x / margin, 0.2, 1.0);
//     safety_multiplier *= clamp((w->width - d->x) / margin, 0.2, 1.0);
//     safety_multiplier *= clamp(d->y / margin, 0.2, 1.0);
//     safety_multiplier *= clamp((w->height - d->y) / margin, 0.2, 1.0);

//     safety_multiplier *= clamp((d->z - 2.0) / 3.0, 0.2, 1.0);
//     safety_multiplier *= clamp((35.0 - d->z) / 10.0, 0.2, 1.0);

//     double obs_pen = fabs(getObstaclePenalty(w, d)); 
//     double hum_pen = fabs(getHumanProximityPenalty(w, d));
//     double env_multiplier = clamp(1.0 - ((obs_pen + hum_pen) / 2.0), 0.1, 1.0); 
//     safety_multiplier *= env_multiplier;

//     return reward * safety_multiplier;
// }

double getReward(Env *env) {
    World *w = env->physical_world;
    Drone *d = w->drone;
    
    // 1. Calcul du score de signal (Objectif principal)
    double average_signal_norm = 0.0;
    double min_signal_norm = getSignalMetrics(w, d, &average_signal_norm);
    double signal_score = (min_signal_norm * 0.5) + (average_signal_norm * 0.5);
    
    // Récompense de base (0.1 pour survivre) + récompense linéaire plafonnée pour le signal.
    // On évite pow() ici pour ne pas créer d'incitation disproportionnée à raser le sol.
    double base_reward = 0.1 + (signal_score * 3.0); 
    
    // 2. Calcul des pénalités strictes (Sécurité)
    // On récupère les valeurs brutes des pénalités (qui étaient calculées dans tes autres fonctions)
    double obs_pen = fabs(getObstaclePenalty(w, d)); 
    double hum_pen = fabs(getHumanProximityPenalty(w, d));
    
    // Nouvelle pénalité fatale d'altitude : s'il vole sous 2 mètres, on lui soustrait des points
    // Plus il s'approche de 0, plus la pénalité est énorme (jusqu'à -4.0)
    double altitude_pen = 0.0;
    if (d->z < 2.0) {
        altitude_pen = 2.0 * (2.0 - d->z); 
    }

    // 3. Pénalités d'éloignement (Bords de carte et plafond)
    double bounds_pen = 0.0;
    double margin = 10.0;
    if (d->x < margin) bounds_pen += (margin - d->x) * 0.05;
    if (d->x > w->width - margin) bounds_pen += (d->x - (w->width - margin)) * 0.05;
    if (d->y < margin) bounds_pen += (margin - d->y) * 0.05;
    if (d->y > w->height - margin) bounds_pen += (d->y - (w->height - margin)) * 0.05;
    
    // On pénalise s'il vole trop haut (inutile pour le signal WiFi)
    if (d->z > 35.0) bounds_pen += (d->z - 35.0) * 0.05;

    // 4. Calcul de la récompense totale soustractive
    // Les pénalités annuleront instantanément tout gain de signal
    double total_reward = base_reward - (obs_pen * 2.0) - (hum_pen * 2.0) - altitude_pen - bounds_pen;

    // Optionnel : borner la récompense maximale/minimale par step (pour la stabilité du réseau)
    return clamp(total_reward, -5.0, 5.0);
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

    // Premiere étape : remplir la grille d'utilisateur
    fillUsersGrid(env, state_out);

    // Deuxième étape : capture des obstacles (LiDAR)
    captureObstacles(env, state_out);

    //Troisième étape : variables physiques du drone (comme avec une target A -> B)
    int drone_offset = GRID_SIZE * GRID_SIZE + (MAX_CLOSEST_OBSTACLES * 3);

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

    double avg_signal = 0.0;
    getSignalMetrics(w, d, &avg_signal);
    state_out[drone_offset + 11] = avg_signal;
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

    if (crashed) { final_reward -= 20.0; }

    *reward = final_reward; 
    *is_terminal = crashed;
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

    // On met d'abord à jour la topologie du monde (sinon les obstacles bougent après le spawn)
    if (current_epoch < 400) {
        majWorld(w, NO_RAND);
    } else if (current_epoch < 800 && current_epoch % 30 == 0) {
        majWorld(w, LOW_RAND);
    } else if (current_epoch % 30 == 0) {
        majWorld(w, TOTAL_RAND);
    }

    // On cherche des coordonnées sûres pour le drone
    double spawn_x, spawn_y, spawn_z;
    getSafeDroneSpawn(w, &spawn_x, &spawn_y, &spawn_z);

    *(w->drone) = createDrone(spawn_x, spawn_y, spawn_z);
    
    env->current_reward = 0.0;
    env->is_terminal = 0;

    getStateVector(env, env->current_state);
}