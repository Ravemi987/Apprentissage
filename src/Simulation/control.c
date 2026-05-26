#include "control.h"

// 200 ticks de 10ms = 2.0 secondes sans toucher le clavier pour que l'IA reprenne la main
#define TIMEOUT_TICKS 200 


void setNonBlockingMode(int enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}


DQNModel* createAgent(World *w, const char *model_path, long *max_battery_ticks) {
    DQNModel *ai = DQNModelCreate(w);

    printf("Chargement du modèle : %s...\n", model_path);
    networkLoad(ai->q_network, model_path);

    Config *cfg = DQNModelGetConfig(ai);
    cfg->epsilon = 0.0;
    cfg->max_steps = 5000;
    
    resetEnv(ai->env, 0); 
    
    // Calcul de l'autonomie en ticks physiques
    *max_battery_ticks = cfg->max_steps * FRAME_SKIP; 
    
    return ai;
}


void readUserInput(Drone *d, int *running, int *timeout_counter, int *current_manual_action) {
    static int key_timeout = 0;
    char ch = 0;
    
    if (read(STDIN_FILENO, &ch, 1) > 0) {
        d->is_autonomous_mode = 0; // Passage en mode manuel
        *timeout_counter = TIMEOUT_TICKS; // Reset du timer
        key_timeout = 15; // Reset timer des touches du clavier (et non du mode manuel)

        if (ch == 'z') *current_manual_action = ENGINE_UP;
        else if (ch == 's') *current_manual_action = ENGINE_DOWN;
        else if (ch == 'q') *current_manual_action = ENGINE_ROLL_LEFT;
        else if (ch == 'd') *current_manual_action = ENGINE_ROLL_RIGHT;
        else if (ch == 'a') *current_manual_action = ENGINE_PITCH_LEFT;
        else if (ch == 'e') *current_manual_action = ENGINE_PITCH_RIGHT;
        else if (ch == 'r') *current_manual_action = ENGINE_YAW_LEFT;
        else if (ch == 'f') *current_manual_action = ENGINE_YAW_RIGHT;
        else if (ch == 'w') *current_manual_action = ENGINE_IDLE;
        else if (ch == 'x') *running = 0;
    } else {
        // Aucune touche lue : décrémentation du timer touche
        if (key_timeout > 0) {
            key_timeout--;
            if (key_timeout == 0) {
                *current_manual_action = ENGINE_IDLE;
            }
        }

        // Aucune touche pressée : décrémentation du timer clavier et reprise IA
        if (*timeout_counter > 0) {
            (*timeout_counter)--;
            if (*timeout_counter == 0) {
                d->is_autonomous_mode = 1;
                *current_manual_action = ENGINE_IDLE;
                handleCommand(d, ENGINE_IDLE); // Mise à plat de sécurité
            }
        }
    }
}


void applyCommand(Drone *d, DQNModel *ai, Env *env, long ticks, int current_manual_action, int *current_ai_action) {
    if (d->is_autonomous_mode == 0) {
        // Mode manuel : itération 100Hz
        handleCommand(d, current_manual_action);
    } else {
        // Mode IA : itération cadencée par le FRAME_SKIP
        if (ticks % FRAME_SKIP == 0) {
            getStateVector(env, env->current_state);
            *current_ai_action = predict(ai, env->current_state);
            handleCommand(d, *current_ai_action);
        }
    }
}


void updatePhysics(World *w, Drone *d, int *running, long ticks, long max_battery_ticks, int current_ai_action, int current_manual_action) {
    physicsStep(w, DT);

    if (isDroneCrashed(w)) {
        printf("\n\n[CRASH] Le drone a touché un obstacle ou le sol ! Fin de la simulation.\n");
        *running = 0;
        return;
    }
    
    if (ticks >= max_battery_ticks) {
        printf("\n\n[INFO] Batterie vide (Max Steps atteints). Fin de la mission.\n");
        *running = 0;
        return;
    }

    if (ticks % 10 == 0) { // Rafraichissement de la console à 10 Hz
        char* mode_str = (d->is_autonomous_mode) ? "\033[32m[ IA ]\033[0m" : "\033[31m[MANUEL]\033[0m";
        int active_action = (d->is_autonomous_mode) ? current_ai_action : current_manual_action;
        double battery_pct = 100.0 - ((double)ticks / (double)max_battery_ticks) * 100.0;
        
        printf("\r%s Batt:%3.0f%% | Action:%d | Pos: X=%5.1f Y=%5.1f Z=%5.1f | R:%5.1f° P:%5.1f° Y:%5.1f°   ", 
               mode_str, battery_pct, active_action, d->x, d->y, d->z, d->phi*57.3, d->theta*57.3, d->psi*57.3);
        fflush(stdout);
    }
}


void exportData(World *w, const char *json_path) {
    exportStateToJSON(w, json_path);
}
