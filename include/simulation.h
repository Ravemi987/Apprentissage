#ifndef __SIMU_H__
#define __SIMU_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "utils.h"

#define GRAVITY 9.81          // G : tire vers le bas sur l'axe y
#define DRONE_MASS 0.5        // 0.5kg
#define ARM_LENGTH 0.15       // 0.15m : Distance entre le centre du drone et les moteurs (influence la vitesse de basculement)
#define THRUST_UNIT 4.0       // 4.0 Newton : Force par moteur. Total: 8N > 0.5 * 9.81 = 4.9 N
#define INERTIA 0.1           // Resistance à la rotation (pour pas que le drone tourne comme une toupie)
#define DT 0.01               // Step de temps en secondes (s) = 100 Hz

#define SIGNAL_BASE_POWER -30.0  // -30.0 dBm : Puissance à 1m
#define PATH_LOSS_EXPONENT 2.0   // Milieu Hertzien


typedef struct {
    // On simule de la 3D en 2D
    double x, y, z;        // Position
    double vx, vy, vz;     // Vitesse
    double theta_roll;     // Inclinaison Gauche/Droite (gère le déplacement sur X)
    double theta_pitch;    // Inclinaison Avant/Arrière (gère le déplacement sur Z)
    double v_roll;         // Vitesse angulaire sur X
    double v_pitch;        // Votesse angulaire sur Z
} Drone;


typedef struct {
    double x, y, z;
} User;


typedef struct {
    Drone *drone;
    User *users;
    int numUsers;
    double width, height, depth;    // Dimensions de la carte
} World;


enum engineThrust {
    ENGINE_NONE,
    ENGINE_FULL,
    ENGINE_LEFT, 
    ENGINE_RIGHT,
    ENGINE_FORWARD,
    ENGINE_BACKWARD
};


void physicsStep(World *w, int action, double dt);

double computeRSSI(Drone* d, User* u);

double getReward(World *w);

void exportStateToJSON(World *w, const char *filepath);

#endif
