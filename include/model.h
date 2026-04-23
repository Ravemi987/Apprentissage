#ifndef __RL_MODEL_H__
#define __RL_MODEL_H__

#include <stdbool.h>
#include <stdio.h>

#include "env.h"
#include "config.h"

typedef struct s_rl_model RLModel;

/* Creation */

RLModel *RLModelCreate(RLEnv *userData);

void RLModelDelete(RLModel **m);

/* Setters */

void RLModelSetConfig(RLModel *m, RLConfig cfg);

/* Getters */

float *RLModelGetStatesValues(RLModel *m);

int *RLModelGetPolicy(RLModel *m);

float *RLModelGetQTable(RLModel *m);

RLConfig *RLModelGetConfig(RLModel *m);

float RLModelGetReward(RLModel *m, RLState s, RLAction a);

RLState RLModelGetNextState(RLModel *m, RLState s, RLAction a);

RLAction RLModelGetBestAction(RLModel *m , RLState s);

/* Print */

void RLModelPrintStatesValues(RLModel *m);

void RLModelPrintPolicy(RLModel *m);

void RLModelPrintQTable(RLModel *m);

/* Algorithms */

void valueIteration(RLModel *m);

void policyIteration(RLModel *m);

void QLearning(RLModel *m);

#endif
