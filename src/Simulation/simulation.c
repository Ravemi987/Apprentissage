#include "simulation.h"


/* Ajoute des contraintes au déplacement (crash, limite de la carte) */
void applyLimits(World *w) {
    Drone *d = w->drone;

    // Sol et Plafond (Axe Y)
    if (d->y < 0) {
        d->y = 0; d->vx = -d->vx * 0.5; d->vy = -d->vy * 0.5; d->vz = -d->vz * 0.5;
        d->theta_roll = 0; d->theta_pitch = 0; // Remise à plat complète
        d->v_roll = 0; d->v_pitch = 0;
    }
    if (d->y > w->height) {
        d->y = w->height; d->vy = -d->vy * 0.5;
    }

    // Murs Gauche / Droit (Axe X)
    if (d->x < 0) {
        d->x = 0; d->vx = -d->vx * 0.5;
    }
    if (d->x > w->width) {
        d->x = w->width; d->vx = -d->vx * 0.5;
    }

    // Murs Avant / Arrière (Axe Z)
    if (d->z < 0) {
        d->z = 0; d->vz = -d->vz * 0.5;
    }
    if (d->z > w->depth) {
        d->z = w->depth; d->vz = -d->vz * 0.5;
    }
}


double computeTotalThrust(int action) {
    double thrustTotal = 0;

    if (action == ENGINE_FULL) {
        thrustTotal = THRUST_UNIT * 4;
    } else if (action != ENGINE_NONE) {
        thrustTotal = THRUST_UNIT * 2;
    }
    
    return thrustTotal;
} 


/* Calcul des accélérations angulaires */
void computeAngularAcceleration(double *alpha_roll, double *alpha_pitch, int action) {
    if (action == ENGINE_LEFT) *alpha_roll = (THRUST_UNIT * ARM_LENGTH) / INERTIA;
    if (action == ENGINE_RIGHT) *alpha_roll = -(THRUST_UNIT * ARM_LENGTH) / INERTIA;
    if (action == ENGINE_FORWARD) *alpha_pitch = (THRUST_UNIT * ARM_LENGTH) / INERTIA;
    if (action == ENGINE_BACKWARD) *alpha_pitch = -(THRUST_UNIT * ARM_LENGTH) / INERTIA;
}



/* Mise à jour des caractéristiques du drone */
void physicsStep(World *w, int action, double dt) {
    Drone *d = w->drone;
    double thrustTotal = computeTotalThrust(action);

    double alpha_roll = 0; 
    double alpha_pitch = 0;

    // Détermination des poussées moteur et accélérations angulaires
    computeAngularAcceleration(&alpha_roll, &alpha_pitch, action);

    // Mise à jour des vitesse angulaires avec amortissement
    d->v_roll = (d->v_roll + alpha_roll * dt) * 0.90;
    d->v_pitch = (d->v_pitch + alpha_pitch * dt) * 0.90;

    // Mise à jour des angles d'inclinaison
    d->theta_roll += d->v_roll * dt;
    d->theta_pitch += d->v_pitch * dt;

    // Projection des forces et calcul des accélérations
    double ax = (thrustTotal * sin(d->theta_roll)) / DRONE_MASS;
    double az = (thrustTotal * sin(d->theta_pitch)) / DRONE_MASS;
    // On prend le cosinus des deux angles pour la perte de portance verticale
    double ay = ((thrustTotal * cos(d->theta_roll) * cos(d->theta_pitch)) - (DRONE_MASS * GRAVITY)) / DRONE_MASS;
    
    // Mise à jour des vitesses avec amortissement
    d->vx = (d->vx + ax * dt) * 0.98;
    d->vy = (d->vy + ay * dt) * 0.98;
    d->vz = (d->vz + az * dt) * 0.98;

    // Mise à jour des positions
    d->x += d->vx * dt; 
    d->y += d->vy * dt;
    d->z += d->vz * dt;

    // Gestion des limites (par exemple le drone se crash)
    applyLimits(w);
}


/* Log-Distance Path Loss Model :
 * La puissance du signal WiFi décroît en fonction de la distance

 * RSSI = Ptx ​− 10 * n * log10​(d)
 * où Ptx est la puissance d'émission, n le coefficient de perte et d la distance
*/
double computeRSSI(Drone *d, User *u) {
    double dist = sqrt(pow(d->x - u->x, 2) + pow(d->y - u->y, 2) + pow(d->z - u->z, 2)) + 0.001;  // On évite log10(0)
    return SIGNAL_BASE_POWER - (10 * PATH_LOSS_EXPONENT * log10(dist));
}


double getReward(World *w) {
    return 0.0;
}


/* Exporte l'état du monde dans un fichier JSON pour l'interface Web */
void exportStateToJSON(World *w, const char *filepath) {
    FILE *f = fopen(filepath, "w");
    if (f == NULL) return;

    fprintf(f, "{\n");
    fprintf(f, "  \"drone\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f, \"roll\": %.4f, \"pitch\": %.4f},\n", 
            w->drone->x, w->drone->y, w->drone->z, w->drone->theta_roll, w->drone->theta_pitch);
    
    fprintf(f, "  \"users\": [\n");
    for (int i = 0; i < w->numUsers; i++) {
        fprintf(f, "    {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}", w->users[i].x, w->users[i].y, w->users[i].z);
        if (i < w->numUsers - 1) fprintf(f, ",\n");
        else fprintf(f, "\n");
    }
    fprintf(f, "  ]\n");
    fprintf(f, "}\n");
    fclose(f);
}
