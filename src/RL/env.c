#include "env.h"

#include <stdlib.h>


int isTerminalState(Env *env) {
    return 0;
}


void envStep(Env *env, State *next_state, double *reward, int *is_terminal, int action_idx) {
    handleCommand(env->physical_world->drone, action_idx);
    physicsStep(env->physical_world, DT);
    getStateVector(env->physical_world, next_state, env->target_x, env->target_y, env->target_z);
    *reward = getReward(env->physical_world);
    *is_terminal = isTerminalState(env);
}


void resetEnv(Env *env) {
    World *w = env->physical_world;
    *(w->drone) = createDrone(w->drone->x, w->drone->y, w->drone->z);
    State s;
    getStateVector(w, &s, env->target_x, env->target_y, env->target_z);
    env->current_reward = 0.0;
    env->is_terminal = 0;
    env->current_state = s;
}


Env *initEnv(World *w) {
    Env *env = malloc(sizeof(Env));
    State s;
    getStateVector(w, &s, env->target_x, env->target_y, env->target_z);
    env->physical_world = w;
    env->current_reward = 0.0;
    env->is_terminal = 0;
    env->current_state = s;

    return env;
}
