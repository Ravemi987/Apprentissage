#include "env.h"

#include <stdlib.h>


int isTerminalState(Env *env) {
    return 0;
}

void envStep(Env *env, State *next_state, double *reward, int *is_terminal, int action_idx) {
    handleCommand(env->physical_world->drone, action_idx);
    physicsStep(env->physical_world, DT);
    normalizeState(next_state, env->physical_world);
    *reward = getReward(env->physical_world);
    *is_terminal = isTerminalState(env);
}
