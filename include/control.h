#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "simulation.h"
#include "model.h"
#include <generation.h>

#define TIMEOUT_TICKS 200

void setNonBlockingMode(int enable); 

DQNModel* createAgent(World *w, const char *model_path, long *max_battery_ticks);

void readUserInput(Drone *d, int *running, int *timeout_counter, int *current_manual_action);

void applyCommand(Drone *d, DQNModel *ai, Env *env, long ticks, int current_manual_action, int *current_ai_action);

void updatePhysics(World *w, Drone *d, int *running, long ticks, long max_battery_ticks, int current_ai_action, int current_manual_action);

void exportData(World *w, const char *json_path);
