#include <stdio.h>
#include <stdlib.h>

#include "agent.h"
#include "action.h"
#include "state.h"


void executeAgent(RLModel *m, RLAgent *agent) {
    RLState currentState = agent->initialState;
    RLState goalState = agent->goalState; 
    
    int steps = 0;

    while (currentState.id != goalState.id && steps < agent->maxIter) {
        RLAction action = RLModelGetBestAction(m, currentState);
        
        action.execute(action.args);
        
        RLState nextState = RLModelGetNextState(m, currentState, action);
        
        currentState = nextState;
        steps++;
    }

    RLAction action = RLModelGetBestAction(m, currentState);
    action.execute(action.args);

    agent->reward = RLModelGetReward(m, currentState, action);
    agent->sucess = (currentState.id == goalState.id);
}
