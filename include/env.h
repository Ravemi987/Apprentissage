#ifndef __RL_ENV_H__
#define __RL_ENV_H__

#include <stdbool.h>
#include <stdio.h>

#include "state.h"
#include "action.h"

typedef struct s_rl_env RLEnv;

RLEnv *RLEnvCreate(int nStates, int nActions,
                   RLState *states, RLAction *actions,
                   int (*Generate)(RLState *, RLAction*, float**, float**));

void RLEnvDelete(RLEnv **env);

RLAction RLEnvGetAction(RLEnv *e, int a);

RLState RLEnvGetState(RLEnv *e, int s);

int RLEnvGetNS(RLEnv *e);

int RLEnvGetNA(RLEnv *e);

float *RLEnvGetTransitionArray(RLEnv *e, int s, int a);

int RLEnvGetTransitionState(RLEnv *e, int s, int a);

float RLEnvGetR(RLEnv *e, int s, int a);

#endif
