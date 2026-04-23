#ifndef __RL_ACTION_H__
#define __RL_ACTION_H__

#include <stdbool.h>
#include <stdio.h>

/*
Interface avec un pointeur vers une fonction anonyme
et un pointeur vers des données dont la structure n'est
connue que par cette fonction. Cette interface est publique car
l'utilisateur définit ses propres actions.
*/
typedef struct s_rl_action {
    int id;
    char *name;
    void (*execute)(void *args);
    void *args;
    //void (*cleanup)(void * args);
} RLAction;

RLAction RLActionCreate(int id, char *name, void (*exec)(void *args), 
                       void *args);

#endif
