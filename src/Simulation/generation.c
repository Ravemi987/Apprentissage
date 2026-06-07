#include "generation.h"
#include <stdlib.h>
#include <math.h>

int seedUse;


/* Permet de donner au réseau les informations sur les obstacles. Voir getStateVector */
void captureObstacles(Env *env, double *state_out) {
    // [Ce code reste inchangé, il était très bien]
    World *w = env->physical_world;
    Drone *d = w->drone;

    int obs_offset = 3;

    for (int i = 0; i < MAX_CLOSEST_OBSTACLES * 3; i++) {
        state_out[obs_offset + i] = 1.0;
    }

    if (w->numObstacles <= 0) return;

    typedef struct  { double dx, dy, dz, dist_sq; } RelObs;
    RelObs *list = malloc(w->numObstacles * sizeof(RelObs));

    for (int i = 0; i < w->numObstacles; i++) {
        list[i].dx = w->obstacles[i].x - d->x;
        list[i].dy = w->obstacles[i].y - d->y;
        list[i].dz = w->obstacles[i].z - d->z;
        list[i].dist_sq = list[i].dx*list[i].dx + list[i].dy*list[i].dy + list[i].dz*list[i].dz;
    }

    for (int i = 0; i < w->numObstacles - 1; i++) {
        for (int j = 0; j < w->numObstacles - i - 1; j++) {
            if (list[j].dist_sq > list[j+1].dist_sq) {
                RelObs temp = list[j];
                list[j] = list[j+1];
                list[j+1] = temp;
            }
        }
    }

    int limit = (w->numObstacles < MAX_CLOSEST_OBSTACLES) ? w->numObstacles : MAX_CLOSEST_OBSTACLES;
    for (int i = 0; i < limit; i++) {
        state_out[obs_offset + i*3 + 0] = list[i].dx / w->width;
        state_out[obs_offset + i*3 + 1] = list[i].dy / w->height;
        state_out[obs_offset + i*3 + 2] = list[i].dz / w->depth;
    }

    free(list);
}


/* Vérifie géométriquement si une position 3D est libre (Obstacles, Users, Murs et Plafond) */
static int isPositionFree(double x, double y, double z, double required_radius, 
                            Obstacle3D *obstacles, int num_placed_obs, 
                            User *users, int num_placed_users, World *w) {
    
    double wall_margin = required_radius + 4.0;

    // Vérification avec les murs et le plafond
    if (x < wall_margin || x > (w->width - wall_margin)) return 0;
    if (y < wall_margin || y > (w->height - wall_margin)) return 0;
    if (z < 0.0 || z > (w->depth - wall_margin)) return 0;

    // Vérification avec les obstacles déjà placés
    for (int i = 0; i < num_placed_obs; i++) {
        if (z >= obstacles[i].z && z <= (obstacles[i].z + obstacles[i].height + 2.0)) {
            double dist_h = sqrt(pow(x - obstacles[i].x, 2) + pow(y - obstacles[i].y, 2));
            if (dist_h < (required_radius + obstacles[i].radius + SAFETY_RADIUS)) return 0;
        }
    }

    // Vérification avec les utilisateurs déjà placés
    for (int i = 0; i < num_placed_users; i++) {
        double dist_h = sqrt(pow(x - users[i].x, 2) + pow(y - users[i].y, 2));
        if (z <= 4.0 && dist_h < (required_radius + 3.0)) return 0; 
    }

    return 1;
}


void placeObstacles(User *users, Obstacle3D *obstacles, int numObstacles, World *w) {
    for (int i = 0; i < numObstacles; i++) {
        int placed = 0;
        while(!placed) {
            double x = ((double)rand() / RAND_MAX) * w->width;
            double y = ((double)rand() / RAND_MAX) * w->height;
            double radius = 3.0 + 2.0 * ((double)rand() / RAND_MAX);
            double h = fmin(30.0, w->depth) * ((double)rand() / RAND_MAX);

            if (isPositionFree(x, y, 0.0, radius, obstacles, i, users, 0, w)) {
                obstacles[i] = (Obstacle3D){x, y, 0.0, radius, h};
                placed = 1;
            }
        }
    }
}

void placeUsers(User *users, Obstacle3D *obstacles, int numUsers, int numObstacles, World *w) {
    for (int i = 0; i < numUsers; i++) {
        int placed = 0;
        while(!placed) {
            double x = ((double)rand() / RAND_MAX) * w->width;
            double y = ((double)rand() / RAND_MAX) * w->height;

            if (isPositionFree(x, y, 0.0, 1.0, obstacles, numObstacles, users, i, w)) {
                users[i] = (User){x, y, 0.0, 0};
                placed = 1;
            }
        }
    }
}


/* Génère des coordonnées d'apparition sécurisées pour le drone */
void getSafeDroneSpawn(World *w, double *out_x, double *out_y, double *out_z) {
    int valid_spawn = 0;
    int security_counter = 0;

    while (!valid_spawn) {
        *out_x = ((double)rand() / RAND_MAX) * w->width;
        *out_y = ((double)rand() / RAND_MAX) * w->height;
        *out_z = 5.0 + ((double)rand() / RAND_MAX) * ((w->depth / 2.0) - 5.0);

        if (isPositionFree(*out_x, *out_y, *out_z, 2.0, w->obstacles, w->numObstacles, w->users, w->numUsers, w)) {
            
            int too_close_to_someone = 0;
            for (int i = 0; i < w->numUsers; i++) {
                double dist = sqrt(pow(*out_x - w->users[i].x, 2) + pow(*out_y - w->users[i].y, 2));
                if (dist < 15.0) {
                    too_close_to_someone = 1;
                    break;
                }
            }
            if (!too_close_to_someone) valid_spawn = 1;
        }

        if (++security_counter > 1000) {
            *out_x = w->width / 2.0; *out_y = w->height / 2.0; *out_z = w->depth - 10.0;
            break;
        }
    }
}


World createWorld(int numUsers, int numObstacles, double width, double height, double depth) {
    User *users = malloc(numUsers * sizeof(User));
    Obstacle3D *obstacles = malloc(numObstacles * sizeof(Obstacle3D));

    World w = {
        .drone = NULL, .users = users, .numUsers = numUsers,
        .obstacles = obstacles, .numObstacles = numObstacles,
        .width = width, .height = height, .depth = depth
    };

    placeObstacles(users, obstacles, numObstacles, &w);
    placeUsers(users, obstacles, numUsers, numObstacles, &w);

    double spawn_x, spawn_y, spawn_z;
    getSafeDroneSpawn(&w, &spawn_x, &spawn_y, &spawn_z);

    w.drone = malloc(sizeof(Drone));
    *(w.drone) = createDrone(spawn_x, spawn_y, spawn_z);

    return w;
}


/* Mise à jour du monde */
void majWorld(World *w, Type_maj_w maj){
    if (maj == TOTAL_RAND) {
        World wBis = createWorld(w->numUsers, w->numObstacles, w->width, w->height, w->depth);
        free(w->users);
        free(w->obstacles);
        w->obstacles = wBis.obstacles;
        w->users = wBis.users;
    }
}
