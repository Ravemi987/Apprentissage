#ifndef __RL_STATE_H__
#define __RL_STATE_H__

#include <stdbool.h>
#include <stdio.h>

typedef struct s_rl_state {
    int id;
    char *label;
    void *context;
} RLState;

#endif
