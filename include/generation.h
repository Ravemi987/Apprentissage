#ifndef __GENERATION_H__
#define __GENERATION_H__

#include <stdlib.h>
#include "simulation.h"
#include "env.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

typedef enum {
    NO_RAND, TOTAL_RAND
} Type_maj_w;


void majWorld(World *w, Type_maj_w maj);

void captureObstacles(Env *env, double *state_out);

void getSafeDroneSpawn(World *w, double *out_x, double *out_y, double *out_z);

World createWorld(int numUsers, int numObstacles, double width, double height, double depth);

#endif
