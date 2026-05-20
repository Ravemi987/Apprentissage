#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "simulation.h"
#include "model.h"

int main() {
    Drone d = createDrone(100.0, 50.0, 10.0);

    User users[] = { 
        {170.0, 80.0, 0.0}, 
        {165.0, 75.0, 0.0},
        {175.0, 85.0, 0.0},
        {180.0, 70.0, 0.0}
    }; 
    int num_users = sizeof(users) / sizeof(users[0]);

    Obstacle3D obstacles[] = {
        {30.0, 20.0, 0.0, 3.0, 15.0},   
        {30.0, 80.0, 0.0, 4.0, 20.0},  
        {100.0, 10.0, 0.0, 2.5, 12.0}
    };
    int num_obstacles = sizeof(obstacles) / sizeof(obstacles[0]);

    World w = { 
        .drone = &d, 
        .users = users, 
        .numUsers = num_users, 
        .obstacles = obstacles,
        .numObstacles = num_obstacles,
        .width = 200.0, 
        .height = 100.0, 
        .depth = 100.0
    };

    char *model_path = "files/drone_wifi_brain.txt";
    char *json_path = "web/state.json";

    printf("==================================================\n");
    printf("     DEMARRAGE DU DRONE EN MODE EVALUATION        \n");
    printf("==================================================\n");

    // 4. Création de l'agent
    // On met un batchSize de 1 ici car on fait uniquement de l'inférence (pas de train)
    DQNModel *ai = DQNModelCreate(&w, 1000, 1, 0.0, 0.0);
    
    // 5. Chargement du cerveau entraîné
    printf("Chargement du modèle : %s...\n", model_path);
    networkLoad(ai->q_network, model_path);

    // 6. Config de test : EPSILON A DEUX ZEROS (0.0)
    // On force l'intelligence pure, aucune action au hasard n'est tolérée
    Config *cfg = DQNModelGetConfig(ai);
    cfg->epsilon = 0.0;
    cfg->epsilon_min = 0.0;
    cfg->max_steps = 5000; // Durée max du vol de démonstration

    printf("Prêt pour le décollage ! Mode 100%% Exploitation.\n");
    printf("Exportation en direct vers : %s\n", json_path);
    printf("--------------------------------------------------\n");

    Env *env = ai->env;
    double next_state[NB_STATES];
    double reward;
    int is_terminal = 0;
    int steps = 0;
    double total_reward = 0.0;

    // Réinitialisation de l'environnement (avec l'époque 0 pour forcer la config de base propre)
    // Si tu as gardé l'ancienne signature sans curriculum, remets juste resetEnv(env);
    resetEnv(env, 0); 

    // Boucle de vol visuelle
    for (steps = 0; steps < env->max_steps; steps++) {
                
        // L'IA observe l'état et choisit la meilleure action absolue
        int action = predict(ai, env->current_state, NULL);

        // Application de l'action physique et calcul du saut de trames (Frame Skip)
        envStep(env, next_state, &reward, &is_terminal, action);
        total_reward += reward;

        // Sauvegarde de l'état du monde dans le JSON pour l'affichage Web
        exportStateToJSON(&w, json_path);

        // Transition vers le nouvel état
        memcpy(env->current_state, next_state, NB_STATES * sizeof(double));

        // Affichage console en direct
        printf("\rStep: %4d/%d | Action: %d | Reward: %6.2f | Pos: X=%5.1f Y=%5.1f Z=%5.1f", 
               steps + 1, env->max_steps, action, reward, w.drone->x, w.drone->y, w.drone->z);
        fflush(stdout);

        usleep(50000); 

        if (is_terminal) {
            printf("\n\n[CRASH] Le drone a touché un obstacle ou le sol ! Fin du test.\n");
            break;
        }
    }

    if (!is_terminal) {
        printf("\n\n[SUCCES] Le drone a accompli tout son temps de vol avec succès !\n");
    }

    printf("Bilan du vol - Étapes survécues: %d | Score Total: %.2f\n", steps, total_reward);
    printf("==================================================\n");

    DQNModelDelete(&ai);
    return 0;
}
