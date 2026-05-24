#include "config.h"
#include <stdlib.h>


Config defaultConfig(void) {
    return (Config){
        .gamma = 0.99,
        .epsilon = 1.0,
        .epsilon_min = 0.0,
        .epsilon_decay = 0.001,
        .epochs = 1500,
        .max_steps = 3000,
        .update_freq = 2000,
        .batch_size = 128,
        .learning_rate = 5e-4,
        .decay = 0.0
    };
}

void epsilonDecay(Config *cfg) {
    if (cfg->epsilon > cfg->epsilon_min) {
        cfg->epsilon *= (1.0 - cfg->epsilon_decay);
        if (cfg->epsilon < cfg->epsilon_min) {
            cfg->epsilon = cfg->epsilon_min;
        }
    }
}
