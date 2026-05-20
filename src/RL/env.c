#include "env.h"
#include <stdlib.h>


/* Fonction utilitaire pour limiter une valeur entre un min et un max (Hard Clip) */
double clamp(double val, double min_val, double max_val) {
    if (isnan(val) || isinf(val)) return 0.0;
    
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}


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
                penalty -= clamp(penetration, 0.0, 1.0) * 1.0; 
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
                penalty -= clamp(penetration, 0.0, 1.0) * 1.0;
            }
        }
    }
    return penalty;
}


/* Calcule la moyenne normalisée du signal pour tous les utilisateurs (0.0 à 1.0) */
static double getAverageSignalNorm(World *w, Drone *d, int *out_connected_count) {
    if (w->numUsers <= 0) return 0.0;
    
    double total_rssi_norm = 0.0;
    *out_connected_count = 0;

    for (int i = 0; i < w->numUsers; i++) {
        double rssi = computeRSSI(d, &w->users[i]);
        if (isnan(rssi) || isinf(rssi)) rssi = -100.0;

        // Normalisation (Pire : -100dBm -> 0.0 | Parfait : -30dBm -> 1.0)
        double norm = clamp((rssi + 100.0) / 70.0, 0.0, 1.0);
        total_rssi_norm += norm;
        
        if (rssi > SIGNAL_BASE_POWER) (*out_connected_count)++;
    }
    
    return total_rssi_norm / w->numUsers;
}


/* Fonction Principale de Récompense */
double getReward(Env *env) {
    World *w = env->physical_world;
    Drone *d = w->drone;
    
    // Pénalité de temps (orce l'agent à être efficace)
    double reward = -0.05; 

    // Gestion du Signal (Absolu + Delta)
    int connected = 0;

    // Attention, garder garder la valeur moyenne et surtout pasfaire une différence !
    double current_signal_norm = getAverageSignalNorm(w, d, &connected);

    if (w->numUsers > 0) {
        // Si le signal est parfait (1.0), il gagne +1.0, ce qui annule la pénalité de temps (-0.05) et encourage le hovering.
        reward += current_signal_norm * 1.0;
        
        if (connected == w->numUsers) reward += 0.05; // Bonus de réussite
    }

    // Application des pénalités environnementales
    reward += getObstaclePenalty(w, d);
    reward += getHumanProximityPenalty(w, d);

    // Pénalité linéaire globale 
    double speed_squared = pow(d->x_dot, 2) + pow(d->y_dot, 2) + pow(d->z_dot, 2);
    reward -= speed_squared * 0.003; 

    // Pénalité angulaire modérée (Pour diminuer le Yaw, très sensible avec un impact invisible, sans interdire de tourner)
    double angular_speed = pow(d->p, 2) + pow(d->q, 2) + pow(d->r, 2);
    reward -= angular_speed * 0.001;

    return reward;
}


/* Permet de donner au réseau les informations sur les utilisateurs. Voir getStateVector */
static void fillUsersGrid(Env *env, double *state_out) {
    World *w = env->physical_world;

    int num_cells = GRID_SIZE * GRID_SIZE;
    for (int i = 0; i < num_cells; i++) state_out[i] = 0.0; // Réinitialisation

    for (int i = 0; i < w->numUsers; i++) {
        // L'index d'une cellule est : la position d'un utilisateur, divisée par la taille d'une cellule
        int cell_x = (int)(w->users[i].x / (w->width / GRID_SIZE));
        int cell_y = (int)(w->users[i].y / (w->height / GRID_SIZE));

        // Sécurité
        if (cell_x < 0) {cell_x = 0;} if (cell_x >= GRID_SIZE) {cell_x = GRID_SIZE - 1;}
        if (cell_y < 0) {cell_y = 0;} if (cell_y >= GRID_SIZE) {cell_y = GRID_SIZE - 1;}

        state_out[cell_y * GRID_SIZE + cell_x] += 1.0; // Mise à jour du nombre d'utilisateurs dans cette cellule
    }

    // Normalisation très importante (comme pour toutes les valeurs)
    for (int i = 0; i < num_cells; i++) {
        if (w->numUsers > 0) state_out[i] /= w->numUsers;
    }
}


/* Permet de donner au réseau les informations sur les obstacles. Voir getStateVector */
static void captureObstacles(Env *env, double *state_out) {
    World *w = env->physical_world;
    Drone *d = w->drone;

    // Décalage par rapport aux infos des users
    int obs_offset = GRID_SIZE * GRID_SIZE;

    // Par défaut, on initialise les 9 valeurs (3 coordonnées pour 3 obstacles) valeurs à 1.0 (obstacles loins, aucun danger)
    for (int i = 0; i < MAX_CLOSEST_OBSTACLES * 3; i++) {
        state_out[obs_offset + i] = 1.0;
    }

    if (w->numObstacles <= 0) return;

    // Structure locale temporaire pour trier les obstacles par distance
    typedef struct  { double dx, dy, dz, dist_sq; } RelObs;
    RelObs *list = malloc(w->numObstacles * sizeof(RelObs));

    for (int i = 0; i < w->numObstacles; i++) {
        list[i].dx = w->obstacles[i].x - d->x;
        list[i].dy = w->obstacles[i].y - d->y;
        list[i].dz = w->obstacles[i].z - d->z;
        list[i].dist_sq = list[i].dx*list[i].dx + list[i].dy*list[i].dy + list[i].dz*list[i].dz;
    }

    // tri à bulle pour extraire les plus proches
    for (int i = 0; i < w->numObstacles - 1; i++) {
        for (int j = 0; j < w->numObstacles - i - 1; j++) {
            if (list[j].dist_sq > list[j+1].dist_sq) {
                RelObs temp = list[j];
                list[j] = list[j+1];
                list[j+1] = temp;
            }
        }
    }

    // Injection des coordonnées relative des obstacles les plus proches (normalisées)
    int limit = (w->numObstacles < MAX_CLOSEST_OBSTACLES) ? w->numObstacles : MAX_CLOSEST_OBSTACLES;
    for (int i = 0; i < limit; i++) {
        state_out[obs_offset + i*3 + 0] = list[i].dx / w->width;
        state_out[obs_offset + i*3 + 1] = list[i].dy / w->height;
        state_out[obs_offset + i*3 + 2] = list[i].dz / w->depth;
    }

    free(list);
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

    state_out[drone_offset + 0] = clamp(d->x / w->width, 0.0, 1.0);
    state_out[drone_offset + 1] = clamp(d->y / w->height, 0.0, 1.0);
    state_out[drone_offset + 2] = clamp(d->z / w->depth, 0.0, 1.0);
    state_out[drone_offset + 3] = clamp(d->x_dot / MAX_VELOCITY, -1.0, 1.0);
    state_out[drone_offset + 4] = clamp(d->y_dot / MAX_VELOCITY, -1.0, 1.0);
    state_out[drone_offset + 5] = clamp(d->z_dot / MAX_VELOCITY, -1.0, 1.0);
    state_out[drone_offset + 6] = clamp(d->phi / ANGLE_LIMIT, -1.0, 1.0);
    state_out[drone_offset + 7] = clamp(d->theta / ANGLE_LIMIT, -1.0, 1.0);
    state_out[drone_offset + 8] = clamp(d->psi / MATH_PI, -1.0, 1.0);
    state_out[drone_offset + 9] = clamp(d->p / MAX_ROT, -1.0, 1.0);
    state_out[drone_offset + 10] = clamp(d->q / MAX_ROT, -1.0, 1.0);
    state_out[drone_offset + 11] = clamp(d->r / MAX_ROT, -1.0, 1.0);
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

    if (crashed) { final_reward -= 100.0; }
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


/* Réinitialise l'environnement en se basant UNIQUEMENT sur les paramètres capturés */
void resetEnv(Env *env, int current_epoch) {
    World *w = env->physical_world;

    // Réinitialisation du drone à sa vraie position d'origine
    *(w->drone) = createDrone(env->spawn_drone_x, env->spawn_drone_y, env->spawn_drone_z);
    env->current_reward = 0.0;
    env->is_terminal = 0;

    if (current_epoch < 200) {
        // Mode Fixe
        for (int i = 0; i < w->numUsers; i++) {
            w->users[i].x = env->spawn_users_x[i];
            w->users[i].y = env->spawn_users_y[i];
        }
        for (int i = 0; i < w->numObstacles; i++) {
            w->obstacles[i].x = env->spawn_obs_x[i];
            w->obstacles[i].y = env->spawn_obs_y[i];
        }
        
    } else if (current_epoch < 500) {
        // Mode Bruit
        double max_noise = 6.0;
        for (int i = 0; i < w->numUsers; i++) {
            double noise_x = (((double)rand() / (double)RAND_MAX) * 2.0 - 1.0) * max_noise;
            double noise_y = (((double)rand() / (double)RAND_MAX) * 2.0 - 1.0) * max_noise;
            w->users[i].x = clamp(env->spawn_users_x[i] + noise_x, 10.0, w->width - 10.0);
            w->users[i].y = clamp(env->spawn_users_y[i] + noise_y, 10.0, w->height - 10.0);
        }
        for (int i = 0; i < w->numObstacles; i++) {
            double noise_x = (((double)rand() / (double)RAND_MAX) * 2.0 - 1.0) * max_noise;
            double noise_y = (((double)rand() / (double)RAND_MAX) * 2.0 - 1.0) * max_noise;
            w->obstacles[i].x = clamp(env->spawn_obs_x[i] + noise_x, 20.0, w->width - 20.0);
            w->obstacles[i].y = clamp(env->spawn_obs_y[i] + noise_y, 20.0, w->height - 20.0);
        }
        
    } else {
        // Mode Aléatoire Total 
        for (int i = 0; i < w->numUsers; i++) {
            w->users[i].x = 10.0 + ((double)rand() / (double)RAND_MAX) * (w->width - 20.0);
            w->users[i].y = 10.0 + ((double)rand() / (double)RAND_MAX) * (w->height - 20.0);
        }
        
        for (int i = 0; i < w->numObstacles; i++) {
            double obs_x, obs_y;
            int valid_placement;
            
            do {
                valid_placement = 1; // On part du principe que c'est bon
                
                obs_x = 20.0 + ((double)rand() / (double)RAND_MAX) * (w->width - 40.0);
                obs_y = 20.0 + ((double)rand() / (double)RAND_MAX) * (w->height - 40.0);
                
                // SÉCURITÉ DRONE : L'obstacle doit être loin du spawn du drone
                double dist_to_drone = sqrt(pow(obs_x - env->spawn_drone_x, 2) + pow(obs_y - env->spawn_drone_y, 2));
                if (dist_to_drone < 25.0) {
                    valid_placement = 0;
                }
                
                //CORRECTION MAJEURE (SÉCURITÉ HUMAINE) : L'obstacle ne doit pas écraser un utilisateur
                // On laisse une marge confortable (Rayon de l'arbre + SAFETY_RADIUS + marge de manoeuvre)
                for (int u = 0; u < w->numUsers; u++) {
                    double dist_to_user = sqrt(pow(obs_x - w->users[u].x, 2) + pow(obs_y - w->users[u].y, 2));
                    double min_clearance = w->obstacles[i].radius + SAFETY_RADIUS + 5.0; 
                    
                    if (dist_to_user < min_clearance) {
                        valid_placement = 0; // Invalide, on rejette cette position
                        break; 
                    }
                }
                
            } while (!valid_placement); // On boucle tant qu'on n'a pas un emplacement 100% sain
            
            w->obstacles[i].x = obs_x;
            w->obstacles[i].y = obs_y;
        }
    }

    getStateVector(env, env->current_state);
}
