#include "config.h"
#include <stdlib.h>


Config defaultConfig(void) {
    return (Config){
        .gamma = 0.99,
        .epsilon = 1.0,
        .epsilon_min = 0.05,
        .epsilon_decay = 0.001,
        .epochs = 5000
    };
}

void configSetGamma(Config *cfg, float v) {
    cfg->gamma = v;
}

void configSetEpsilon(Config *cfg, float v) {
    cfg->epsilon = v;
}

void configSetEpochs(Config *cfg, int v) {
    cfg->epochs = v;
}

void epsilonDecay(Config *cfg) {
    if (cfg->epsilon > cfg->epsilon_min) {
        cfg->epsilon *= (1.0 - cfg->epsilon_decay);
        if (cfg->epsilon < cfg->epsilon_min) {
            cfg->epsilon = cfg->epsilon_min;
        }
    }
}
