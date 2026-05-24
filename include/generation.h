#ifndef __GENERATION_H__
#define __GENERATION_H__

#include <stdlib.h>
#include "simulation.h"
#include "env.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// si la seed est -1, on en genere une aleatoire, sinon, on prends cette seed

/**
 * @brief Fonction permettant de generer le monde dans lequels le drone va evoluer
 * On cadrie le monde puis on creer une entite dans chaque case, si trop de case, on ne remplie pas forcement la case.
 * @param drone Le drone qui servira pour la simulation
 * @param numUsers Le nombre d'utilisateurs a generer
 * @param numObstacles Le nombre d'obstacle a generer
 * @param width Largeur du monde
 * @param height Hauteur du monde
 * @param depth Longeur du monde
 * @param seed Seed a utilisere pour la generation aleatoire (reproductiblite possible)
 */
World creationWorld(Drone *drone, int numUsers, int numObstacles, double width, double height, double depth, int seed);

typedef enum {
    NO_RAND, LOW_RAND, TOTAL_RAND
} Type_maj_w;

/**
 * @brief Fonction qui permet de faire evoluer le monde en fonction du nombre de setp d'entraiement.
 * On ne change rien si current_epoch < 200
 * On fait de leger changements (deplacement leger des structures dans les memes "cases") si current_epoch < 500
 * On fait de gros changements (deplacement des structures entre les cases) si current_epoch > 500
 * @param world Le monde a modifier
 * @param type_maj_w La manière de mettre a jour le monde
 */
void majWorld(World *w, Type_maj_w maj);


#endif