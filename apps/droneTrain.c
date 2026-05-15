#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

    // Initialisation de la physique
    Drone d = createDrone(100.0, 50.0, 10.0);
    User users[2] = { {40, 40, 0}, {160, 60, 0} }; 
    World w = { .drone = &d, .users = users, .numUsers = 2, .width = 200, .height = 100, .depth = 100 };

    // Point cible (Au centre de la carte, à 50m de haut)
    double target[3] = {100.0, 50.0, 50.0};

    // Initialisation de l'IA (Hyperparamètres)
    int update_freq = 1000;
    int batch_size = 64;
    double learning_rate = 0.001;
    double decay = 0.0;
    
    char *model_path = "drone_brain.bin";

    printf("Création de l'agent DQN...\n");
    DQNModel *ai = DQNModelCreate(&w, update_freq, batch_size, learning_rate, decay, target);
    DQNModelSetPath(ai, model_path);

    if (fileExists(model_path)) {
        printf("Modèle existant trouvé ! Reprise de l'entraînement...\n");
        networkLoad(ai->q_network, model_path);
        networkCopyWeights(ai->target_network, ai->q_network); // On synchronise le clone
        ai->config.epsilon = 0.1; // Si on reprend, on baisse l'exploration (10% max)
    } else {
        printf("Aucun modèle trouvé. Démarrage d'un nouvel entraînement.\n");
    }

    DeepQLearning(ai);

    // Sauvegarde finale
    networkSave(ai->q_network, model_path);
    printf("=== ENTRAINEMENT TERMINE ===\n");

    DQNModelDelete(&ai);
    return 0;
}