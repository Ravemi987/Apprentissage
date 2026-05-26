#include "config.h"
#include <stdlib.h>


Config defaultConfig(void) {
    return (Config){
        .gamma = 0.95,
        .epsilon = 1.0,
        .epsilon_min = 0.0,
        .epsilon_decay = 0.004,
        .epochs = 1500,
        .max_steps = 2000,
        .update_freq = 5000,
        .batch_size = 128,
        .learning_rate = 1e-3,
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
