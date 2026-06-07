#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
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
    Drone d = createDrone(100.0, 100.0, 20.0);

    int width = 200.0;
    int height = 200.0;
    int depth = 100.0;

    int seed = 1234;
    int num_users = 1;
    int num_obstacles = 10;

    // Assemblage du monde 3D
    World w = createWorld(&d, num_users, num_obstacles, width, height, depth, seed);
    
    printf("==================================================\n");
    printf("Création de l'agent DQN pour la Couverture Réseau...\n");
    printf("Population : %d utilisateurs | Obstacles : %d arbres\n", w.numUsers, w.numObstacles);
    printf("==================================================\n");
    
    char *model_path = "files/drone_wifi_brain.txt";

    DQNModel *ai = DQNModelCreate(&w);
    DQNModelSetPath(ai, model_path);

    int start_epoch = 0;

    if (fileExists(model_path)) {
        printf("Modèle existant trouvé ! Reprise de l'entraînement...\n");
        modelLoad(ai, &start_epoch, &seed);
    } else {
        printf("Aucun modèle trouvé. Démarrage d'un nouvel entraînement WiFi (Table rase).\n");
    }

    // Lancement de la boucle Deep-Q-Learning
    DeepQLearning(ai, start_epoch, seed);

    // Sauvegarde finale
    modelSave(ai, defaultConfig().epochs, seed);

    printf("\n=== ENTRAINEMENT TERMINE AVEC SUCCES ===\n");

    DQNModelDelete(&ai);
    return 0;
}
