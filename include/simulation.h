#ifndef __SIMU_H__
#define __SIMU_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "utils.h"

// NE PAS MODIFIER
#define G 9.81  // Gravité
#define M 2.5   // Masse du drone en kg
#define L 0.112 // Longueur de chaque bras de levier en m
#define DT 0.01 // Pas de temps en secondes (s) = 100 Hz
#define Ixx 5e-3    // Moment d'inertie en kg.m2
#define Iyy 5e-3    // Moment d'inertie en kg.m2
#define Izz 10e-3   // Moment d'inertie en kg.m2
#define B 1.5   // Coefficient de poussée/thrust
#define D 1.3   // Coefficient de traînée/drag
#define RHO 1.225 // Densité de l'air en kg/m3
#define RADIUS 0.0635 // Rayon des hélices en m
#define MATH_PI 3.14159265358979323846 // PI
#define ANGLE_LIMIT 0.3 // Limite pour éviter les singularités de gimbal lock
#define MAX_ROT 5.0 // Vitesse de rotation maximale en rad/s
#define MAX_VELOCITY 30.0
#define DRAG_COEFF (0.5 * RHO * 0.1 * D * MATH_PI * pow(RADIUS, 2)) // Coefficient de traînée aérodynamique

#define SIGNAL_BASE_POWER -30.0  // -30.0 dBm : Puissance à 1m
#define PATH_LOSS_EXPONENT 2.0   // Milieu Hertzien

#define SAFETY_RADIUS 2.0   // Distance de sécurité avec les objets

#define NB_ACTION 9

typedef struct {
    double kp;
    double ki;
    double kd;
    double integral;
    double prev_error;
} PIDController;


typedef struct {
    double x, y, z; // Position
    double x_dot, y_dot, z_dot; // Vitesse linéaire
    double x_dot_dot, y_dot_dot, z_dot_dot; // Accélération linéaire
    double phi, theta, psi; // Angles roll, pitch et yaw
    double phi_dot, theta_dot, psi_dot; // Dérivées des angles
    double p, q, r; // Vitesse angulaire
    double p_dot, q_dot, r_dot; // Dérivées des vitesses angulaires
    double omega[4]; // Vitesses de rotation des moteurs

    double target_thrust;
    double target_roll;
    double target_pitch;
    double target_yaw;

    PIDController pid_roll;
    PIDController pid_pitch;
    PIDController pid_yaw;

} Drone;


typedef enum {
    ENGINE_IDLE = 0,
    ENGINE_UP,
    ENGINE_DOWN,
    ENGINE_PITCH_LEFT,
    ENGINE_PITCH_RIGHT,
    ENGINE_ROLL_LEFT,
    ENGINE_ROLL_RIGHT,
    ENGINE_YAW_LEFT,
    ENGINE_YAW_RIGHT
} EngineAction;


typedef struct {
    double x, y, z;
} User;


typedef struct {
    double x, y, z;
    double radius;  // Rayon
    double height;  // Hauteur
} Obstacle3D;


typedef struct {
    Drone *drone;
    User *users;
    Obstacle3D *obstacles;
    int numUsers;
    int numObstacles;
    double width, height, depth;    // Dimensions de la carte. Attention, depth est la vraie hauteur ! (axe Z vers le haut)
} World;


Drone createDrone(double x, double y, double z);

void handleCommand(Drone *d, int action);

void physicsStep(World *w, double dt);

int isDroneCrashed(World *w);

double computeRSSI(Drone* d, User* u);

void exportStateToJSON(World *w, const char *filepath);

int collisionWithUser(World *w);

#endif
