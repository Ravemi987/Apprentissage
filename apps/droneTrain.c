#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h> // Pour vérifier si le fichier existe
#include "simulation.h"
#include "model.h"
#include <generation.h>

// Fonction utilitaire pour vérifier si un fichier existe
int fileExists(const char *filename) {
    struct stat buffer;   
    return (stat(filename, &buffer) == 0);
}

int main() {
    srand(time(NULL));

    // Initialisation des structutres
    Drone d = createDrone(100.0, 50.0, 10.0);

    int width = 200.0;
    int height = 200.0;
    int depth = 100.0;

    int seed = 1234;
    int num_users = 4;
    int num_obstacles = 3;

    // Assemblage du monde 3D
    World w = creationWorld(&d, num_users, num_obstacles, width, height, depth, seed);

    // Initialisation des hyperparamètres de l'IA (DQN)
    int update_freq = 2000;
    int batch_size = 128;
    double learning_rate = 5e-4;
    double decay = 0.0;
    
    char *model_path = "files/drone_wifi_brain.txt";

    printf("==================================================\n");
    printf("Création de l'agent DQN pour la Couverture Réseau...\n");
    printf("Population : %d utilisateurs | Obstacles : %d arbres\n", w.numUsers, w.numObstacles);
    printf("==================================================\n");

    DQNModel *ai = DQNModelCreate(&w, update_freq, batch_size, learning_rate, decay);
    // Config *cfg = DQNModelGetConfig(ai);
    // cfg->epsilon = 0.596;

    DQNModelSetPath(ai, model_path);

    if (fileExists(model_path)) {
        printf("Modèle existant trouvé ! Reprise de l'entraînement...\n");
        networkLoad(ai->q_network, model_path);
        networkCopyWeights(ai->target_network, ai->q_network); // Synchronisation
    } else {
        printf("Aucun modèle trouvé. Démarrage d'un nouvel entraînement WiFi (Table rase).\n");
    }

    // Lancement de la boucle Deep-Q-Learning
    DeepQLearning(ai);

    // Sauvegarde finale
    // rajouter la sauvegarde de la seed ici.
    networkSave(ai->q_network, model_path);
    printf("\n=== ENTRAINEMENT TERMINE AVEC SUCCES ===\n");

    DQNModelDelete(&ai);
    return 0;
}
