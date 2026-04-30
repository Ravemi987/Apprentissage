#include "simulation.h"


/* Ajoute des contraintes au déplacement (crash, limite de la carte) */
void applyLimits(World *w) {
    Drone *d = w->drone;

    // Sol
    if (d->y < 0) {
        d->y = 0; d->vx = -d->vx * 0.5; d->vy = -d->vy * 0.5;
        d->theta = 0; d->v_theta = 0;
    }

    // Plafond
    if (d->y > w->height) {
        d->y = w->height;
        d->vy = -d->vy * 0.5;
    }

    // Mur Gauche
    if (d->x < 0) {
        d->x = 0;
        d->vx = -d->vx * 0.5;
    }

    // Mur Droit
    if (d->x > w->width) {
        d->x = w->width;
        d->vx = -d->vx * 0.5;
    }
}


/* Ajoute des forces de frottements */
void applyDamping(World *w) {
    Drone *d = w->drone;

    d->v_theta *= 0.95;
    d->vx *= 0.99;
    d->vy *= 0.99;
}


/* Mise à jour des caractéristiques du drone */
void physicsStep(World *w, int action, double dt) {
    Drone *d = w->drone;

    // Détermination des poussées moteur
    double thrustL = (action == ENGINE_LEFT || action == ENGINE_FULL) ? THRUST_UNIT : 0;
    double thrustR = (action == ENGINE_RIGHT || action == ENGINE_FULL) ? THRUST_UNIT : 0;
    double totalT = thrustL + thrustR;

    double alpha = ((thrustL - thrustR) * ARM_LENGTH) / INERTIA;  // Calcul de l'accélération angulaire
    d->v_theta += alpha * dt;;    // Mise à jour de la vitesse angulaire
    d->theta += d->v_theta * dt;  // Mise à jour de l'angle d'inclinaison

    // Calcul des accélérations, mise à jour des vitesses et positions
    double ax = (totalT * sin(d->theta)) / DRONE_MASS;
    double ay = ((totalT * cos(d->theta)) - (DRONE_MASS * GRAVITY)) / DRONE_MASS;
    d->vx += ax * dt; d->vy += ay * dt;
    d->x += d->vx * dt; d->y += d->vy * dt;


    // Gestion des limites (par exemple le drone se crash)
    applyLimits(w);

    // Ajoute des forces de frottements. On le fait à la fin pour ne pas fausser les calculs intermédiaires
    applyDamping(w);
}


/* Log-Distance Path Loss Model :
 * La puissance du signal WiFi décroît en fonction de la distance

 * RSSI = Ptx ​− 10 * n * log10​(d)
 * où Ptx est la puissance d'émission, n le coefficient de perte et d la distance
*/
double computeRSSI(Drone *d, User *u) {
    double dist = sqrt(pow(d->x - u->x, 2) + pow(d->y - u->y, 2)) + 0.001;  // On évite log10(0)
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
    // Drone (x, y, theta)
    fprintf(f, "  \"drone\": {\"x\": %.2f, \"y\": %.2f, \"theta\": %.4f},\n", 
            w->drone->x, w->drone->y, w->drone->theta);
    
    // Utilisateurs
    fprintf(f, "  \"users\": [\n");
    for (int i = 0; i < w->numUsers; i++) {
        fprintf(f, "    {\"x\": %.2f, \"y\": %.2f}", w->users[i].x, w->users[i].y);
        if (i < w->numUsers - 1) fprintf(f, ",\n");
        else fprintf(f, "\n");
    }
    fprintf(f, "  ]\n");
    fprintf(f, "}\n");

    fclose(f);
}
