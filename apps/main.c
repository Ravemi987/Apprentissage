#include "control.h"

int main() {
    srand(time(NULL));

    Drone d = createDrone(100.0, 50.0, 10.0);
    World w = creationWorld(&d, 4, 3, 200.0, 200.0, 100.0, 1234);

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
    int timeout_counter = 0;
    int current_manual_action = ENGINE_IDLE;
    int current_ai_action = ENGINE_IDLE;
    
    d.is_autonomous_mode = 1;

    while (running) {
        
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
