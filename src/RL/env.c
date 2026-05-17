#include "env.h"
#include <stdlib.h>


/* Détermine si oui ou non on a terminé */
int isTerminalState(Env *env, int step_count) {
    World *w = env->physical_world;

    // Crash
    if (isDroneCrashed(w)) {
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

    // Maximiser le RSSI moyen des utilisateurs
    double total_rssi = 0.0;
    int connected = 0;

    for (int i = 0; i < w->numUsers; i++) {
        double rssi = computeRSSI(d, &w->users[i]);
        if (isnan(rssi) || isinf(rssi)) rssi = -100; // Sécurité

        total_rssi += rssi; 
        
        if (rssi > SIGNAL_BASE_POWER) connected++;
    }

    if (w->numUsers > 0) reward += ((total_rssi / w ->numUsers) + 100.0) * 0.1;  // RSSI moyen
    if (connected == w->numUsers && w->numUsers > 0) reward += 2.0; // Utilisateurs connectés

    // Protection contre les obstacles
    for (int i = 0; i < w->numObstacles; i++) {
        Obstacle3D obs = w->obstacles[i];

        // Détermine si le drone est à hauteur d'obstacle
        if (d->z >= obs.z && d->z <= (obs.z + obs.height)) {
            double dist_h = sqrt(pow(d->x - obs.x, 2) + pow(d->y - obs.y, 2)); // Distance horizontale
            double safety_zone = obs.radius + SAFETY_RADIUS; 
            if (dist_h < safety_zone) { // Collision
                reward -= (safety_zone - dist_h) * 1.5; 
            }
        }
    }

    // Protection contre les utilisateurs
    for (int i = 0; i < w->numUsers; i++) {
        User u = w->users[i];
        if (d->z >= 0.0 && d->z <= 4.0) {
            double dist_h = sqrt(pow(d->x - u.x, 2) + pow(d->y - u.y, 2));
            if (dist_h < SAFETY_RADIUS) {
                reward -= (SAFETY_RADIUS - dist_h) * 2.0; // Punition forte (priorité humaine)
            }
        }
    }

    // Pénalité de temps
    reward -= 0.05;

    // Pénalité d'excès de vitesse (magnitude de la vitesse au carré)
    double speed_squared = pow(d->x_dot, 2) + pow(d->y_dot, 2) + pow(d->z_dot, 2);
    reward -= speed_squared * 0.001;

    // MANQUE COLLISION UTILISATEUR

    if (isDroneCrashed(w)) {
        reward -= 500.0;
    }

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


/* Fonction permettant de réinitialiser l'environnement à sont état d'origine après chaque epoch */
void resetEnv(Env *env) {
    World *w = env->physical_world;

    *(w->drone) = createDrone(100.0, 50.0, 20.0);
    env->current_reward = 0.0;
    env->is_terminal = 0;

    getStateVector(env, env->current_state);
}


/* Initialise l'environnement */
Env *initEnv(World *w, int max_steps) {
    Env *env = malloc(sizeof(Env));

    env->physical_world = w;
    env->current_reward = 0.0;
    env->is_terminal = 0;
    env->max_steps = max_steps;

    getStateVector(env, env->current_state);

    return env;
}
