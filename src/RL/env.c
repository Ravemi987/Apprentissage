#include "env.h"

#include <stdlib.h>

/* 
Environnement avec un tableau de transition (3D)
et un tableau de récompenses (1D).
Il contient aussi les fonctions d'accès à cet environnement.
Un environnement est une interface dont l'utilisateur n'a pour le moment qu'à
implanter la fonction de génération des tableaux de données. Plus tard, les fonctions
de transitions et de reward seront aussi dynamiques.
*/
struct s_rl_env {
    // Dimensions des données
    int nS;
    int nA;

    // Tableaux en entrée
    RLState *states;
    RLAction *actions;

    // Tableaux de données
    float *transitions;
    float *rewards;

    // Fonction de générations des tableaux de données
    int (*Generate)(RLState *, RLAction*, float**, float**);
};


/* 
Génération des données de l'environnement.
Fonction implantée par l'utilisateur
 */
int RLEnvInit(RLEnv *env) {
    return env->Generate(
        env->states, env->actions,
        &(env->transitions), &(env->rewards)
    );
}

/*
Créé un environnement
*/
RLEnv *RLEnvCreate(int nStates, int nActions,
                   RLState *states, RLAction *actions,
                   int (*Generate)(RLState *, RLAction*, float**, float**)) {
    RLEnv *env = malloc(sizeof(struct s_rl_env));

    // Dimensions
    env->nS = nStates;
    env->nA = nActions;

    // Tableaux en entrée
    env->states = states;
    env->actions = actions;

    // Tableaux de données
    env->transitions = NULL;
    env->rewards = NULL;

    // Fonction de génération des données
    env->Generate = Generate;

    if (RLEnvInit(env) == -1) return NULL;

    return env;
}

/*
Libère un environnement
*/
void RLEnvDelete(RLEnv **env) {
    if ((*env) == NULL) return;
    free((*env)->transitions);
    free((*env)->rewards);
    free(*env);
    *env = NULL;
}

/* Getters */

int RLEnvGetNS(RLEnv *e) {
    return e->nS;
}

int RLEnvGetNA(RLEnv *e) {
    return e->nA;
}

RLAction RLEnvGetAction(RLEnv *e, int a) {
    return e->actions[a];
} 

RLState RLEnvGetState(RLEnv *e, int s) {
    return e->states[s];
}

/*
Prend en entrée un env avec une matrice 2D: la récompense pour chaque état et action
Retourne la récompense.
*/
float RLEnvGetR(RLEnv *e, int s, int a) {
    return e->rewards[s * e->nA + a];
}

/*
Prend en entrée un environnement.
Retourne un tableau 1D: les probabilités d'atteindre les nouveaux états.
*/
float *RLEnvGetTransitionArray(RLEnv *e, int s, int a) {
    int offset = (s * e->nA * e->nS) + (a * e->nS);
    return &(e->transitions[offset]);
}

/*
Prend en entrée un environnement.
Retourne le nouvel état choisit (celui avec la plus haute probabilité).
*/
int RLEnvGetTransitionState(RLEnv *e, int s, int a) {
    int offset = (s * e->nA * e->nS) + (a * e->nS);

    float max = -1.0f;
    int newState = 0;

    for (int nextState = 0; nextState < e->nS; ++nextState) {
        float v = e->transitions[offset + nextState];
        if (v > 0.0f && v >= max) {
            max = v;
            newState = nextState;
        }
    }

    return newState;
}
