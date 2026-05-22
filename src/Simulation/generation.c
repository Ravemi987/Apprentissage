#include "generation.h"
#include <stdlib.h>

int seedUse;

World creationWorld(Drone *drone, int numUsers, int numObstacles, double width, double height, double depth, int seed) {
    // initialisation aleatoire
    if (seed == -1) {
        seedUse = time(NULL); // mettre une genearation aleatoire
    } else {
        seedUse = seed;
    }
    srand(seedUse);


    // Creation d'une "grille" contenant ((numUser + numObstacles)/2)**2 cases 
    int taille = (numObstacles + numUsers);
    int table[numObstacles + numUsers][2];

    // amelioration avec l'ajout d'un grille plus la fonction for du milieu
    for (int i = 0; i < numObstacles+numUsers; i++) {
        int found = 0;
        while(!found) {
            int x = rand() % taille;
            int y = rand() % taille;

            int dejaPresent = 0;
            for (int j = 0; j < i; j++) { // On cherche uniquement parmi les éléments déjà placés
                if (table[j][0] == x && table[j][1] == y) {
                    dejaPresent = 1;
                    break; // Pas la peine de continuer à chercher, on l'a trouvé
                }
            }
            if (!dejaPresent) {
                table[i][0] = x;
                table[i][1] = y;
                found = 1;
            }
        }
    }

    int userRestant = numUsers;
    int obstRestant = numObstacles;
    User *users = malloc(numUsers * sizeof(User));
    Obstacle3D *obstacles = malloc(numObstacles * sizeof(Obstacle3D));

    float tailleCaseX = (float)(width / taille);
    float tailleCaseY = (float)(depth / taille); 

    // Iteration sur la grille pour placer tous les obstacles
    for (int indice = 0; indice < numObstacles + numUsers; indice++) {
        int i = table[indice][0];
        int j = table[indice][1];
        // on place un elmeent 
        if (userRestant > 0) {
            // on place un utilisateur
            float x =  tailleCaseX * ((float)rand()/(float)RAND_MAX + (float)i);
            float y =  tailleCaseY * ((float)rand()/(float)RAND_MAX + (float)j);
            // toujours au sol
            users[userRestant - 1] = (User){x, y, 0.0};
            userRestant -= 1;
        } else {
            // on place un obstacle
            float x =  tailleCaseX * ((float)rand()/(float)RAND_MAX + (float)i);
            float y =  tailleCaseY * ((float)rand()/(float)RAND_MAX + (float)j);
            float radius = 5.0 * (float)(rand()/RAND_MAX);
            float hauteur = MIN(30.0, height) * (float)rand()/(float)RAND_MAX;

            obstacles[obstRestant - 1] = (Obstacle3D){x, y, 0.0, radius, hauteur};
            obstRestant -= 1;


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

    return w;
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
    case TOTAL_RAND :
        // redefinition de la map.
        World wBis = creationWorld(w->drone, w->numUsers, w->numObstacles, w->width, w->height, w->depth, seed);
        free(w->users);
        free(w->obstacles);
        w->obstacles = wBis.obstacles;
        w->users = wBis.users;
        break;
    
    default:
        break;
    }

}

void lowDeplacementMAj(World *w, int seed) {

    float taille = w->numUsers + w->numObstacles;


    float tailleCaseX = (float)(w->width / taille);
    float tailleCaseY = (float)(w->depth / taille);

    for (int i; i < w->numUsers; i++) {
        float x = w->users[i].x;
        float y = w->users[i].y;

        int xInt = (int)(x/tailleCaseX);
        int yInt = (int)(y/tailleCaseY);

        w->users[i].x = ((float)xInt + (float)rand()/(float)RAND_MAX) * tailleCaseX;
        w->users[i].y = ((float)yInt + (float)rand()/(float)RAND_MAX) * tailleCaseY;
    }
    // on deplace selement un peu les obstacles, on ne les mets pas a jours 
    for (int i; i < w->numUsers; i++) {
        float x = w->obstacles[i].x;
        float y = w->obstacles[i].y;

        int xInt = (int)(x/tailleCaseX);
        int yInt = (int)(y/tailleCaseY);

        w->obstacles[i].x = ((float)xInt + (float)rand()/(float)RAND_MAX) * tailleCaseX;
        w->obstacles[i].y = ((float)yInt + (float)rand()/(float)RAND_MAX) * tailleCaseY;
    }
    
}

