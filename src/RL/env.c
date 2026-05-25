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
static double getSignalMetrics(World *w, Drone *d, int *out_connected_count, double *out_average_norm) {
    if (w->numUsers <= 0) {
        *out_connected_count = 0;
        *out_average_norm = 0.0;
        return 0.0;
    }

    double min_norm = 1.0; // On commence au maximum possible pour trouver le pire
    double total_norm = 0.0;
    *out_connected_count = 0;

    for (int i = 0; i < w->numUsers; i++) {
        // Un seul calcul de RSSI par utilisateur !
        double rssi = computeRSSI(d, &w->users[i]);
        if (isnan(rssi) || isinf(rssi)) rssi = -100.0;

        // Normalisation (Pire : -100dBm -> 0.0 | Parfait : -30dBm -> 1.0)
        double norm = clamp((rssi + 100.0) / 70.0, 0.0, 1.0);
        
        // Accumulation pour la moyenne
        total_norm += norm;

        // Sauvegarde du pire signal (Min)
        if (norm < min_norm) {
            min_norm = norm;
        }

        // Compteur de connexion globale
        if (rssi > -55) {
            (*out_connected_count)++;
        }
    }

    // On extrait la moyenne via le pointeur de sortie
    *out_average_norm = total_norm / w->numUsers;

    // On retourne le min de manière classique
    return min_norm;
}

 
// /* Fonction Principale de Récompense */
// /* Fonction Principale de Récompense Calibrée */
// double getReward(Env *env) {
//     World *w = env->physical_world;
//     Drone *d = w->drone;
    
//     int connected = 0;
//     double average_signal_norm = 0.0;

//     // Récupération des métriques (Assure-toi d'avoir mis : if (rssi >= -80.0) dans getSignalMetrics)
//     double min_signal_norm = getSignalMetrics(w, d, &connected, &average_signal_norm);
    
//     double reward_mix = (min_signal_norm * 0.8) + (average_signal_norm * 0.2);
//     double reward = reward_mix * 2.0; 

//     // Ne donne pas de bonus si le drone est en train de raser le sol ou de frôler un obstacle
//     if (d->z >= 4.0 && !collisionWithUser(w) && !collisionWithObstacle(w)) {
//         // Le signal minimal est exploitable (Évite de se focaliser sur un seul user au détriment des autres)
//         if (min_signal_norm > 0.5) reward += 0.3;
//         // Tout le monde est connecté
//         if (w->numUsers > 0 && connected == w->numUsers) reward += 0.5;
//     }

//     // Pénalités environnementales
//     reward += getObstaclePenalty(w, d); 
//     reward += getHumanProximityPenalty(w, d); 

//     if (d->z < 4.0) {
//         double penetration = (4.0 - d->z) / 4.0;
//         reward -= clamp(penetration, 0.0, 1.0) * 1.5;
//     }

//     // Pénalité des limites de la carte
//     double edge_margin = 10.0;
//     if (d->x < edge_margin)  reward -= (edge_margin - d->x) / edge_margin * 0.5;
//     if (d->x > w->width - edge_margin)  reward -= (d->x - (w->width - edge_margin)) / edge_margin * 0.5;
//     if (d->y < edge_margin)  reward -= (edge_margin - d->y) / edge_margin * 0.5;
//     if (d->y > w->height - edge_margin) reward -= (d->y - (w->height - edge_margin)) / edge_margin * 0.5;

//     // Pénalités cinématiques
//     double speed_squared = pow(d->x_dot, 2) + pow(d->y_dot, 2) + pow(d->z_dot, 2);
//     reward -= speed_squared * 0.001; 
    
//     double angular_speed = pow(d->p, 2) + pow(d->q, 2) + pow(d->r, 2);
//     reward -= angular_speed * 0.0015;

//     return reward;
// }

/* Fonction Principale de Récompense - VERSION MATHÉMATIQUEMENT PARFAITE */
double getReward(Env *env) {
    World *w = env->physical_world;
    Drone *d = w->drone;
    
    int connected = 0;
    double average_signal_norm = 0.0;
    double min_signal_norm = getSignalMetrics(w, d, &connected, &average_signal_norm);
    
    // 1. COMPOSANTE PRINCIPALE (Entre 0.0 et 1.0)
    // 50% min (équilibre l'essaim pour ne délaisser personne) / 50% moyenne (couverture globale)
    double signal_score = (min_signal_norm * 0.5) + (average_signal_norm * 0.5);
    
    // 2. MULTIPLICATEUR DE SÉCURITÉ (Entre 0.0 et 1.0)
    // Remplace les soustractions. Garantit que la récompense ne sera JAMAIS négative.
    double safety_multiplier = 1.0;
    
    // 2.a - Altitude (Baisse la récompense s'il vole sous les 4m)
    if (d->z < 3.5) {
        safety_multiplier *= clamp(d->z / 4.0, 0.0, 1.0);
    }
    
    // 2.b - Limites de carte (Baisse la récompense s'il s'approche à moins de 10m des bords)
    double margin = 10.0;
    if (d->x < margin) safety_multiplier *= clamp(d->x / margin, 0.0, 1.0);
    else if (d->x > w->width - margin) safety_multiplier *= clamp((w->width - d->x) / margin, 0.0, 1.0);
    
    if (d->y < margin) safety_multiplier *= clamp(d->y / margin, 0.0, 1.0);
    else if (d->y > w->height - margin) safety_multiplier *= clamp((w->height - d->y) / margin, 0.0, 1.0);

    // 2.c - Humains et Obstacles (Conversion de tes pénalités en multiplicateur)
    // Tes fonctions renvoient entre 0.0 et -2.0. On prend la valeur absolue.
    double obs_pen = fabs(getObstaclePenalty(w, d)); 
    double hum_pen = fabs(getHumanProximityPenalty(w, d));
    double env_multiplier = clamp(1.0 - ((obs_pen + hum_pen) / 2.0), 0.0, 1.0);
    safety_multiplier *= env_multiplier;

    // 3. CALCUL DE LA RÉCOMPENSE DE BASE (Strictement >= 0)
    // Si safety_multiplier = 0 (danger de mort), reward = 0
    double reward = (signal_score * 2.0) * safety_multiplier;

    // 4. BONUS CONDITIONS (Seulement s'il est parfaitement en sécurité : safety > 0.9)
    if (safety_multiplier > 0.9) {
        if (min_signal_norm > 0.4) reward += 0.3;
        if (w->numUsers > 0 && connected == w->numUsers) reward += 0.5;
    }

    // TAXE CINÉMATIQUE (Multiplicateur pour éviter les valeurs négatives)
    // Punit les tremblements et les rotations abusives
    double speed_sq = pow(d->x_dot, 2) + pow(d->y_dot, 2) + pow(d->z_dot, 2);
    double ang_sq = pow(d->p, 2) + pow(d->q, 2) + pow(d->r, 2);
    double kinetic_penalty = clamp((speed_sq + ang_sq) * 0.0005, 0.0, 0.5);
    
    reward = reward * (1.0 - kinetic_penalty);

    // RÉSULTAT: La récompense est mathématiquement bloquée dans l'intervalle [0.0 , ~2.8].
    return reward;
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

    // "Distance de sécurité sol" : 1.0 si le drone est en sécurité,  et tend vers 0.0 si le drone approche dangereusement du sol (Z < 4m)
    double ground_clearance = d->z / SAFETY_RADIUS;
    state_out[drone_offset + 0] = clamp(ground_clearance, 0.0, 1.0);

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

    if (crashed) { final_reward -= 5.0; }
    if (action_idx == ENGINE_YAW_LEFT || action_idx == ENGINE_YAW_RIGHT) { final_reward -= 0.2; }
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