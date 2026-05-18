#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h> // Pour vérifier si le fichier existe
#include "simulation.h"
#include "model.h"

// Fonction utilitaire pour vérifier si un fichier existe
int fileExists(const char *filename) {
    struct stat buffer;   
    return (stat(filename, &buffer) == 0);
}

int main() {
    srand(time(NULL));

    // Initialisation de la physique du drone
    Drone d = createDrone(100.0, 50.0, 10.0);

    // Définition de la population (Le nombre d'utilisateurs s'adapte automatiquement)
    User users[] = { 
        {40.0, 40.0, 0.0}, 
        {160.0, 60.0, 0.0},
        {100.0, 30.0, 0.0},
        {80.0, 70.0, 0.0}
    }; 
    int num_users = sizeof(users) / sizeof(users[0]);

    // Définition des obstacles 3D (Arbres ou bâtiments : x, y, z, rayon, hauteur)
    Obstacle3D obstacles[] = {
        {60.0, 50.0, 0.0, 3.0, 15.0},   // Arbre 1
        {140.0, 45.0, 0.0, 4.0, 20.0},  // Arbre 2
        {100.0, 80.0, 0.0, 2.5, 12.0}   // Arbre 3
    };
    int num_obstacles = sizeof(obstacles) / sizeof(obstacles[0]);

    // Assemblage du monde 3D complet
    World w = { 
        .drone = &d, 
        .users = users, 
        .numUsers = num_users, 
        .obstacles = obstacles,
        .numObstacles = num_obstacles,
        .width = 200.0, 
        .height = 100.0, 
        .depth = 100.0  // Rappel : altitude maximale du ciel
    };

    // Initialisation des hyperparamètres de l'IA (DQN)
    int update_freq = 1000;
    int batch_size = 64;
    double learning_rate = 1e-5; // Reprise à 1e-4 suite aux sécurités RSSI anti-NaN
    double decay = 0.0;
    
    char *model_path = "files/drone_wifi_brain.txt";

    printf("==================================================\n");
    printf("Création de l'agent DQN pour la Couverture Réseau...\n");
    printf("Population : %d utilisateurs | Obstacles : %d arbres\n", w.numUsers, w.numObstacles);
    printf("==================================================\n");

    DQNModel *ai = DQNModelCreate(&w, update_freq, batch_size, learning_rate, decay);
    DQNModelSetPath(ai, model_path);

    // Ajustement de la configuration pour la version finale
    Config *cfg = DQNModelGetConfig(ai);
    cfg->epochs = 2000;
    cfg->max_steps = 5000;
    cfg->epsilon_decay = 0.002;
    cfg->epsilon_min = 0.05;

    if (fileExists(model_path)) {
        printf("Modèle existant trouvé ! Reprise de l'entraînement...\n");
        networkLoad(ai->q_network, model_path);
        networkCopyWeights(ai->target_network, ai->q_network); // Synchronisation
        cfg->epsilon = 0.50; // On force l'epsilon à 0.50 pour l'exploration modérée
    } else {
        printf("Aucun modèle trouvé. Démarrage d'un nouvel entraînement WiFi (Table rase).\n");
        cfg->epsilon = 1.0;  // Exploration totale par défaut
    }

    // Lancement de la boucle Deep-Q-Learning
    DeepQLearning(ai);

    // Sauvegarde finale
    networkSave(ai->q_network, model_path);
    printf("\n=== ENTRAINEMENT TERMINE AVEC SUCCES ===\n");

    DQNModelDelete(&ai);
    return 0;
}
