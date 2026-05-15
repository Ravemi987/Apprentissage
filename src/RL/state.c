#include "state.h"
#include "simulation.h";


void getStateVector(World *w, State *s, double target_x, double target_y, double target_z) {
    Drone *d = w->drone;

    // Position relative normalisée entre -1 et 1
    s->features[0] = (target_x - d->x) / w->width;
    s->features[1] = (target_y - d->y) / w->height;
    s->features[2] = (target_z - d->z) / w->depth;

    // Vitesses linéaires normalisées entre -1 et 1
    s->features[3] = d->x_dot / MAX_VELOCITY;
    s->features[4] = d->y_dot / MAX_VELOCITY;
    s->features[5] = d->z_dot / MAX_VELOCITY;

    // Angles (déjà normalisés entre -1 et 1 car bornés à [-pi, pi])
    s->features[6] = d->phi;
    s->features[7] = d->theta;
    s->features[8] = d->psi;

    // Vitesses angulaires normalisées entre -1 et 1
    s->features[9] = d->p / MAX_ROT;
    s->features[10] = d->q / MAX_ROT;
    s->features[11] = d->r / MAX_ROT;
}
