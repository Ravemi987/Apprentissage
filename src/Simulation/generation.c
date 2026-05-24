#include "generation.h"
#include <stdlib.h>

int seedUse;

/* Permet de donner au réseau les informations sur les utilisateurs. Voir getStateVector */
void fillUsersGrid(Env *env, double *state_out) {
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

    // Iteration pour placer les personnes
    for (int indice = 0; indice < numUsers; indice++) {
        int nonPlac = 1;
        while(nonPlac == 1) {
            int x = (int)((float)rand()/(float)RAND_MAX * width);
            int y = (int)((float)rand()/(float)RAND_MAX * height);
            // on place un elmeent 
            if (isEmpty(x, y, 1, plan, (int)height, (int)width) == 1) {

                users[indice] = (User){(float)x, (float)y, 0.0};
                // on bloque la case de l'utilisateur
                plan[x * (int)height + y] = 1;

                nonPlac = 0;
            }
        }

    }

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

        w->users[i].x += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->users[i].y += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->users[i].x = MIN(width, MAX(0,w->users[i].x));
        w->users[i].y = MIN(height, MAX(0,w->users[i].y));
    }
    // on deplace selement un peu les obstacles, on ne les mets pas a jours 
    for (int i=0; i < w->numObstacles; i++) {

        w->obstacles[i].x += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->obstacles[i].y += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->obstacles[i].x = MIN(width, MAX(0,w->obstacles[i].x));
        w->obstacles[i].y = MIN(height, MAX(0,w->obstacles[i].y));
    } 
}

void majWorld(World *w, Type_maj_w maj){
    switch (maj)
    {
    case NO_RAND:
        // on ne change pas le monde
        break;
    case LOW_RAND:
        // Deplacement des obstacles/utilisateurs dans leurs cases.
        lowDeplacementMAj(w);
        
        break;
    case TOTAL_RAND : {
        // redefinition de la map.
        World wBis = creationWorld(w->drone, w->numUsers, w->numObstacles, w->width, w->height, w->depth, 0);
        free(w->users);
        free(w->obstacles);
        w->obstacles = wBis.obstacles;
        w->users = wBis.users;
        break; }
    
    default:
        break;
    }
}
