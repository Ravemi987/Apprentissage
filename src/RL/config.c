#include "config.h"
#include <stdlib.h>


Config defaultConfig(void) {
    return (Config){
        .gamma = 0.2,
        .epsilon = 1e-4,
        .epsilon_min = 1.e-4,
        .epsilon_decay = 0.0,
        .alpha = 1,
        .steps = 10,
        .epochs = 100
    };
}

void configSetGamma(Config *cfg, float v) {
    cfg->gamma = v;
}

void configSetEpsilon(Config *cfg, float v) {
    cfg->epsilon = v;
}

void configSetAlpha(Config *cfg, float v) {
    cfg->alpha = v;
}

void configSetSteps(Config *cfg, int v) {
    cfg->steps = v;
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