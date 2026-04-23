#ifndef __AGEN_IMPL_UN__H__
#define __AGEN_IMPL_UN__H__

#include <stdio.h>
#include <stdlib.h>

#include "model.h"
#include "utils.h"
#include "action.h"
#include "state.h"
#include "agent.h"


typedef void (*ActionFunc)(void *context);


void moveLeft(void *args);

void moveUp(void *args);

void moveRight(void *args);

void eat(void *args);

int generateSimpleMatrices(RLState *sArray, RLAction*aArray,
                                   float **T, float **R);

#endif
