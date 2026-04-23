#include <stdlib.h>

#include "action.h"

RLAction RLActionCreate(int id, char *name, void (*exec)(void *args), 
                       void *args) {
    return (RLAction) {
        .id = id,
        .execute = exec,
        .args = args,
        .name = name
    };
}
