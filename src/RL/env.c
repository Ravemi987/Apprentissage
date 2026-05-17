#include "env.h"
#include <stdlib.h>


/* Fonction utilitaire pour limiter une valeur entre un min et un max (Hard Clip) */
double clamp(double val, double min_val, double max_val) {
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
                // Pénalité proportionnelle à l'intrusion (max -0.5 par obstacle)
                penalty -= clamp((safety_zone - dist_h) / safety_zone, 0.0, 1.0) * 0.5; 
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
                // Punition forte si intrusion dans le rayon humain (max -0.8 par humain)
                double penetration = (SAFETY_RADIUS - dist_h) / SAFETY_RADIUS;
                penalty -= clamp(penetration, 0.0, 1.0) * 0.8;
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
    double current_signal_norm = getAverageSignalNorm(w, d, &connected);
    
    // Calcul du Gradient
    double delta_signal = current_signal_norm - env->previous_rssi_norm;
    
    // Mise à jour de la mémoire pour l'étape suivante
    env->previous_rssi_norm = current_signal_norm;

    if (w->numUsers > 0) {
        // Si le signal est parfait (1.0), il gagne +0.1, ce qui annule la pénalité de temps (-0.05) et encourage le hovering.
        reward += current_signal_norm * 0.1;
        
        if (connected == w->numUsers) reward += 0.05; // Bonus de réussite
        
        // Récompense de Gradient pour déplacement dans la bonne direction
        reward += delta_signal * 1.5; 
    }

    // Application des pénalités environnementales
    reward += getObstaclePenalty(w, d);
    reward += getHumanProximityPenalty(w, d);

    // Pénalités de vol (Excès de vitesse et Crash)
    double speed_squared = pow(d->x_dot, 2) + pow(d->y_dot, 2) + pow(d->z_dot, 2);
    reward -= speed_squared * 0.001;

    if (isDroneCrashed(w)) { reward -= 2.0; }

    // CLIPPING (Sécurité pour l'équation de Bellman)
    return clamp(reward, -1.0, 1.0);
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

    state_out[drone_offset + 0] = d->x / w->width;
    state_out[drone_offset + 1] = d->y / w->height;
    state_out[drone_offset + 2] = d->z / w->depth;
    state_out[drone_offset + 3] = d->x_dot / MAX_VELOCITY;
    state_out[drone_offset + 4] = d->y_dot / MAX_VELOCITY;
    state_out[drone_offset + 5] = d->z_dot / MAX_VELOCITY;
    state_out[drone_offset + 6] = d->phi;
    state_out[drone_offset + 7] = d->theta;
    state_out[drone_offset + 8] = d->psi;
    state_out[drone_offset + 9] = d->p / MAX_ROT;
    state_out[drone_offset + 10] = d->q / MAX_ROT;
    state_out[drone_offset + 11] = d->r / MAX_ROT;
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
        // On vérifie si le drone s'est crashé pendant le saut
        crashed = isDroneCrashed(w);
        // On cumule la récompense à chaque mini-étape
        accumulated_reward += getReward(env);
        // Si on se crashe au milieu du Frame Skip, on arrête la boucle
        if (crashed) break;
    }

    getStateVector(env, next_state);

    // On fait la moyenne de la récompense accumulée sur les frames skippées pour garder des valeurs stables pour le réseau de neurones
    *reward = accumulated_reward / FRAME_SKIP; 

    *is_terminal = crashed;
}


/* Fonction permettant de réinitialiser l'environnement à sont état d'origine après chaque epoch */
void resetEnv(Env *env) {
    World *w = env->physical_world;

    *(w->drone) = createDrone(100.0, 50.0, 20.0);
    env->current_reward = 0.0;
    env->is_terminal = 0;

    int dummy_conn;
    env->previous_rssi_norm = getAverageSignalNorm(w, w->drone, &dummy_conn);

    getStateVector(env, env->current_state);
}


/* Initialise l'environnement */
Env *initEnv(World *w, int max_steps) {
    Env *env = malloc(sizeof(Env));

    env->physical_world = w;
    env->current_reward = 0.0;
    env->is_terminal = 0;
    env->max_steps = max_steps;

    int dummy_conn;
    env->previous_rssi_norm = getAverageSignalNorm(w, w->drone, &dummy_conn);

    getStateVector(env, env->current_state);

    return env;
}
