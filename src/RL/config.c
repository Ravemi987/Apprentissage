#include "config.h"
#include <stdlib.h>


RLConfig RLDefaultConfig(void) {
    return (RLConfig){
        .gamma = 0.2,
        .epsilon = 1e-4,
        .alpha = 1,
        .steps = 10,
        .epochs = 100
    };
}

void RLConfigSetGamma(RLConfig *cfg, float v) {
    cfg->gamma = v;
}

void RLConfigSetEpsilon(RLConfig *cfg, float v) {
    cfg->epsilon = v;
}

void RLConfigSetAlpha(RLConfig *cfg, float v) {
    cfg->alpha = v;
}

void RLConfigSetSteps(RLConfig *cfg, int v) {
    cfg->steps = v;
}

void RLConfigSetEpochs(RLConfig *cfg, int v) {
    cfg->epochs = v;
}
