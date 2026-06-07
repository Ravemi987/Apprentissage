#include "control.h"

int checkForResetFlag() {
    if (access("reset.flag", F_OK) == 0) {
        unlink("reset.flag");
        return 1;
    }
    return 0;
}


void resetSimulation(Drone *d, long *ticks, int *timeout_counter, int *current_manual_action, int *current_ai_action) {
    *ticks = 0;
    *timeout_counter = 0;
    *current_manual_action = ENGINE_IDLE;
    *current_ai_action = ENGINE_IDLE;
    d->is_autonomous_mode = 1;
}


int main() {
    int initial_seed = 1234;
    srand(initial_seed);

    double drone_start_x = 100.0;
    double drone_start_y = 100.0;
    double drone_start_z = 20.0;


    Drone d = createDrone(drone_start_x,drone_start_y, drone_start_z);
    World w = createWorld(&d, 4, 10, 200.0, 200.0, 100.0, initial_seed);

    char *json_path = "web/state.json";
    char *model_path = "files/drone_wifi_brain.txt";

    long max_battery_ticks = 0;
    DQNModel *ai = createAgent(&w, model_path, &max_battery_ticks);
    Env *env = ai->env;

    printf("==================================================\n");
    printf("  DEMARRAGE DU DRONE : HYBRID CONTROL (MANUAL/AI) \n");
    printf("==================================================\n");
    printf("\nContrôles : [z/s] Alt | [q/d] Roll | [a/e] Pitch | [r/f] Yaw | [w] Hover | [x] Quitter\n\n");

    setNonBlockingMode(1);

    int running = 1;
    long ticks = 0;
    int timeout_counter, current_manual_action, current_ai_action;

    resetSimulation(&d, &ticks, &timeout_counter, &current_manual_action, &current_ai_action);

    while (running) {

        if (checkForResetFlag()) {
            printf("\n[WEB INTERFACE] Demande de nouvelle map reçue ! Réinitialisation...\n");
            majWorld(&w, NO_RAND);            
            d = createDrone(drone_start_x,drone_start_y, drone_start_z);
            
            resetSimulation(&d, &ticks, &timeout_counter, &current_manual_action, &current_ai_action);
            
            exportData(&w, json_path);
        }
        
        readUserInput(&d, &running, &timeout_counter, &current_manual_action);
        
        if (!running) break; // Sortie immédiate si 'x' pressé

        applyCommand(&d, ai, env, ticks, current_manual_action, &current_ai_action);
        
        updatePhysics(&w, &d, &running, ticks, max_battery_ticks, current_ai_action, current_manual_action);
        
        if (ticks % 4 == 0) {
            exportData(&w, json_path);
        }

        usleep(10000); // 100Hz
        ticks++;
    }

    setNonBlockingMode(0);
    DQNModelDelete(&ai);
    
    return 0;
}
