#include "generation.h"
#include <stdlib.h>

int seedUse;

void bloquer(int x, int y, int radius, int *plan, int depth, int width) {
    for (int i = x-radius; i <= x+radius; i++) {
        for (int j = y-radius; j <= y+radius; j++) {
            if (i >= 0 && i < width && j >= 0 && j < depth) {
                plan[i*depth + j] = 1;
            }
        }
    }
}

World creationWorld(Drone *drone, int numUsers, int numObstacles, double width, double height, double depth, int seed) {
    // initialisation aleatoire
    if (seed == -1) {
        seedUse = time(NULL); // mettre une genearation aleatoire
    } else {
        seedUse = seed;
    }
    srand(seedUse);


    // Creation d'une "grille" contenant ((numUser + numObstacles)/2)**2 cases 
    // ou faire des cases de 1m**2 et 1m seul obstacle par case
    int *plan = calloc((int)width * (int)depth, sizeof(int));

    User *users = malloc(numUsers * sizeof(User));
    Obstacle3D *obstacles = malloc(numObstacles * sizeof(Obstacle3D));


    // Iteration sur la grille pour placer tous les obstacles
    for (int indice = 0; indice < numObstacles; indice++) {
        int nonPlac = 1;
        while(nonPlac) {
            int x = (int)((float)rand()/(float)RAND_MAX * width);
            int y = (int)((float)rand()/(float)RAND_MAX * depth);
            // on place un elmeent 
            if (plan[x * (int)depth + y] == 0) {
                // on place un obstacle
                float radius = 5.0 * (float)rand()/MAX((float)RAND_MAX, 1.0);
                float hauteur = MIN(30.0, height) * (float)rand()/MAX((float)RAND_MAX, 1.0);

                obstacles[indice] = (Obstacle3D){x, y, 0.0, radius, hauteur};
                // on bloque les cases
                bloquer(x, y, (int)radius, plan, (int)depth, (int)width);
                // pour bloquer une seul case
                // plan[x * (int)depth + y] = 1;

                nonPlac = 0;
            }
        }

    }

    // Iteration pour placer les perssones
    for (int indice = 0; indice < numUsers; indice++) {
        int nonPlac = 1;
        while(nonPlac) {
            int x = (int)((float)rand()/(float)RAND_MAX * width);
            int y = (int)((float)rand()/(float)RAND_MAX * depth);
            // on place un elmeent 
            if (plan[x * (int)depth + y] == 0) {

                users[indice] = (User){x, y, 0.0};
                // on bloque la case de l'utilisateur
                plan[x * (int)depth + y] = 1;

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

void lowDeplacementMAj(World *w, int seed) {
    // initialisation aleatoire
    if (seed == -1) {
        seedUse = time(NULL); // mettre une genearation aleatoire
    } else {
        seedUse = seed;
    }
    srand(seedUse);

    float width = w->width;
    float depth = w->depth;

    for (int i=0; i < w->numUsers; i++) {


        w->users[i].x += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->users[i].y += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->users[i].x = MIN(width, MAX(0,w->users[i].x));
        w->users[i].y = MIN(depth, MAX(0,w->users[i].y));
    }
    // on deplace selement un peu les obstacles, on ne les mets pas a jours 
    for (int i=0; i < w->numObstacles; i++) {


        w->obstacles[i].x += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->obstacles[i].y += (((float)rand()/(float)RAND_MAX)-1)*((float)rand()/(float)RAND_MAX)*5;
        w->obstacles[i].x = MIN(width, MAX(0,w->obstacles[i].x));
        w->obstacles[i].y = MIN(depth, MAX(0,w->obstacles[i].y));
    }
    
}

void majWorld(World *w, Type_maj_w maj, int seed){


    switch (maj)
    {
    case NO_RAND:
        // on ne change pas le monde
        break;
    case LOW_RAND:
        // Deplacement des obstacles/utilisateurs dans leurs cases.
        lowDeplacementMAj(w, seed);
        
        break;
    case TOTAL_RAND : {
        // redefinition de la map.
        World wBis = creationWorld(w->drone, w->numUsers, w->numObstacles, w->width, w->height, w->depth, seed);
        free(w->users);
        free(w->obstacles);
        w->obstacles = wBis.obstacles;
        w->users = wBis.users;
        break; }
    
    default:
        break;
    }

}
