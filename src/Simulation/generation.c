#include "generation.h"
#include <stdlib.h>

int seedUse;

void fillUsersGrid(Env *env, double *state_out) {
    World *w = env->physical_world;
    Drone *d = w->drone;

    int num_cells = GRID_SIZE * GRID_SIZE;
    for (int i = 0; i < num_cells; i++) state_out[i] = 0.0; 

    // On définit la portée visuelle du radar du drone (ici infinie après tests)
    double radar_range = MAX(w->width, w->height); 
    
    // Si GRID_SIZE = 4, on a une grille de 4x4 centrée sur le drone.
    // Chaque case du radar représente 20m x 20m (car diamètre de 80m / 4 = 20)
    double cell_size = (radar_range * 2.0) / GRID_SIZE; 

    for (int i = 0; i < w->numUsers; i++) {
        // Calcul de la position de l'utilisateur relative au drone
        double dx = w->users[i].x - d->x;
        double dy = w->users[i].y - d->y;

        double cos_yaw = cos(-d->psi);
        double sin_yaw = sin(-d->psi);
        double local_x = dx * cos_yaw - dy * sin_yaw;
        double local_y = dx * sin_yaw + dy * cos_yaw;

        // On décale pour que (0,0) soit le coin en haut à gauche de notre radar
        double shifted_x = local_x + radar_range;
        double shifted_y = local_y + radar_range;

        // Si l'utilisateur est à portée du radar
        if (shifted_x >= 0 && shifted_x < radar_range * 2.0 &&
            shifted_y >= 0 && shifted_y < radar_range * 2.0) {
            
            int cell_x = (int)(shifted_x / cell_size);
            int cell_y = (int)(shifted_y / cell_size);
            
            // Sécurité
            if (cell_x >= GRID_SIZE) cell_x = GRID_SIZE - 1;
            if (cell_y >= GRID_SIZE) cell_y = GRID_SIZE - 1;

            state_out[cell_y * GRID_SIZE + cell_x] += 1.0; 
        }
    }

    // Normalisation
    for (int i = 0; i < num_cells; i++) {
        if (w->numUsers > 0) state_out[i] /= w->numUsers;
    }
}


/* Permet de donner au réseau les informations sur les obstacles. Voir getStateVector */
void captureObstacles(Env *env, double *state_out) {
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


void bloquer(int x, int y, int radius, int *plan, int height, int width) {
    for (int i = x-radius; i <= x+radius; i++) {
        for (int j = y-radius; j <= y+radius; j++) {
            if (i >= 0 && i < width && j >= 0 && j < height) {
                plan[i*height + j] = 1;
            }
        }
    }
}


int isEmpty(int x, int y, int radius, int *plan, int height, int width) {
    for (int i = x-radius; i <= x+radius; i++) {
        for (int j = y-radius; j <= y+radius; j++) {
            if (i >= 0 && i < width && j >= 0 && j < height) {
                if (plan[i*height + j] == 1) {
                    return 0;
                }
            }
        }
    }
    return 1;
}


void placeObstaclesOnGrid(Obstacle3D *obstacles, int *plan, int numObstacles, double width, double height, double depth) {
    for (int indice = 0; indice < numObstacles; indice++) {
        int nonPlac = 1;
        while(nonPlac == 1) {
            int x = (int)((float)rand()/(float)RAND_MAX * width);
            int y = (int)((float)rand()/(float)RAND_MAX * height);
            // on place un obstacle
            double radius = 3.0 + 2.0 * (double)rand()/MAX((double)RAND_MAX, 1.0);
            double hauteur = MIN(30.0, depth) * (double)rand()/MAX((double)RAND_MAX, 1.0);
            // on place un elmeent 
            if (isEmpty(x, y, (int)radius + 2, plan, (int)height, (int)width) == 1) {
                obstacles[indice] = (Obstacle3D){(double)x, (double)y, 0.0, radius, hauteur};
                // on bloque les cases
                bloquer(x, y, (int)radius + 2, plan, (int)height, (int)width);
                // pour bloquer une seul case
                // plan[x * (int)depth + y] = 1;

                nonPlac = 0;
            }
        }
    }
}


void placeUsersOnGrid(User *users, int *plan, int numUsers, double width, double height) {
    for (int indice = 0; indice < numUsers; indice++) {
        int nonPlac = 1;
        while(nonPlac == 1) {
            int x, y;
        
            x = (int)((float)rand() / (float)RAND_MAX * width);
            y = (int)((float)rand() / (float)RAND_MAX * height);

            // On place l'élément s'il n'y a pas d'obstacle
            if (isEmpty(x, y, 1, plan, (int)height, (int)width) == 1) {
                users[indice] = (User){(float)x, (float)y, 0.0};
                // On bloque la case de l'utilisateur
                plan[x * (int)height + y] = 1;
                nonPlac = 0;
            }
        }
    }
}


/* Génère des coordonnées d'apparition sécurisées pour le drone (Évite les obstacles) */
void getSafeDroneSpawn(World *w, double *out_x, double *out_y, double *out_z) {
    int valid_spawn = 0;
    *out_z = 10.0; // Hauteur de base sécurisée

    while (!valid_spawn) {
        *out_x = 20.0 + ((double)rand() / RAND_MAX) * (w->width - 40.0);
        *out_y = 20.0 + ((double)rand() / RAND_MAX) * (w->height - 40.0);
        
        valid_spawn = 1; 
        
        for (int i = 0; i < w->numObstacles; i++) {
            Obstacle3D obs = w->obstacles[i];
            
            if (*out_z >= obs.z && *out_z <= (obs.z + obs.height)) {
                double dist_h = sqrt(pow(*out_x - obs.x, 2) + pow(*out_y - obs.y, 2));
                
                if (dist_h <= (obs.radius + 1.0)) { // +1.0m de marge
                    valid_spawn = 0; 
                    break; 
                }
            }
        }
    }
}

World creationWorld(Drone *drone, int numUsers, int numObstacles, double width, double height, double depth, int seed) {
    // initialisation aleatoire
    if (seed == -1) {
        seedUse = time(NULL); // mettre une genearation aleatoire
        srand(seedUse);
    } else if (seed != 0) {
        seedUse = seed;
        srand(seedUse);
    }
    
    // Creation d'une "grille" contenant ((numUser + numObstacles)/2)**2 cases 
    // ou faire des cases de 1m**2 et 1m seul obstacle par case
    int *plan = calloc((int)width * (int)height, sizeof(int));
    User *users = malloc(numUsers * sizeof(User));
    Obstacle3D *obstacles = malloc(numObstacles * sizeof(Obstacle3D));

    // On bloque direct la case du drone avec un rayon de sécurité
    int drone_spawn_x = (int)drone->x;
    int drone_spawn_y = (int)drone->y;
    int safety_radius_spawn = 6; 
    bloquer(drone_spawn_x, drone_spawn_y, safety_radius_spawn, plan, (int)height, (int)width);

    // Iteration sur la grille pour placer tous les obstacles
    placeObstaclesOnGrid(obstacles, plan, numObstacles, width, height, depth);

    // Iteration pour placer les personnes
    placeUsersOnGrid(users, plan, numUsers, width, height);

    World w = {
        .drone = drone,
        .users = users,
        .numUsers = numUsers,
        .obstacles = obstacles,
        .numObstacles = numObstacles,
        .width = width,
        .height = height,
        .depth = depth
    };
    free(plan);

    return w;
}


void lowDeplacementMAj(World *w) {
    float width = w->width;
    float height = w->height;

    for (int i=0; i < w->numUsers; i++) {
        float rand_dir = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        float rand_dist = ((float)rand() / (float)RAND_MAX) * 5.0f;
        w->users[i].x += rand_dir * rand_dist;

        rand_dir = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        rand_dist = ((float)rand() / (float)RAND_MAX) * 5.0f;
        w->users[i].y += rand_dir * rand_dist;

        w->users[i].x = MIN(width, MAX(0,w->users[i].x));
        w->users[i].y = MIN(height, MAX(0,w->users[i].y));
    }
    // on deplace selement un peu les obstacles, on ne les mets pas a jours 
    for (int i=0; i < w->numObstacles; i++) {
        float rand_dir = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        float rand_dist = ((float)rand() / (float)RAND_MAX) * 5.0f;
        w->obstacles[i].x += rand_dir * rand_dist;

        rand_dir = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        rand_dist = ((float)rand() / (float)RAND_MAX) * 5.0f;
        w->obstacles[i].y += rand_dir * rand_dist;


        w->obstacles[i].x = MIN(width, MAX(0,w->obstacles[i].x));
        w->obstacles[i].y = MIN(height, MAX(0,w->obstacles[i].y));
    } 
}


void majWorld(World *w, Type_maj_w maj){
    switch (maj) {
    case NO_RAND:
        // on ne change pas le monde
        break;
    case LOW_RAND:
        // Deplacement des obstacles/utilisateurs dans leurs cases.
        lowDeplacementMAj(w);
        
        break;
    case TOTAL_RAND : {
        // redefinition de la map.
        World wBis = creationWorld(w->drone, w->numUsers, w->numObstacles, w->width, w->height, w->depth, -1);
        free(w->users);
        free(w->obstacles);
        w->obstacles = wBis.obstacles;
        w->users = wBis.users;
        break; 
    }
    
    default:
        break;
    }
}
