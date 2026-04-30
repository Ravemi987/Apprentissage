#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "simulation.h"


// Configuration du terminal pour une lecture instantanée
void setNonBlockingMode(int enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // Désactive le buffer de ligne et l'affichage des touches
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // Mode non-bloquant
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

int main() {
    // Initialisation du monde et du drone
    Drone d = { .x = 50, .y = 50, .vx = 0, .vy = 0, .theta = 0, .v_theta = 0 };
    User users[2] = { {20, 0}, {80, 0} }; // Deux utilisateurs au sol
    World w = { .drone = &d, .users = users, .numUsers = 2, .width = 500, .height = 200 };

    setNonBlockingMode(1);
    printf("Pilotez le drone : [q] Gauche | [d] Droite | [z] Full | [s] Rien | [x] Quitter\n");

    char ch;
    int action = ENGINE_NONE;
    int running = 1;

    while (running) {
        // Lecture de la touche
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == 'q') action = ENGINE_LEFT;
            else if (ch == 'd') action = ENGINE_RIGHT;
            else if (ch == 'z') action = ENGINE_FULL;
            else if (ch == 's') action = ENGINE_NONE;
            else if (ch == 'x') running = 0;
        }

        // Mise à jour physique
        physicsStep(&w, action, 0.01);

        // Calcul des signaux pour l'affichage
        double r1 = computeRSSI(&d, &users[0]);
        double r2 = computeRSSI(&d, &users[1]);

        // Affichage dynamique
        printf("\rPos: (%.1f, %.1f) | Ang: %4.1f° | RSSI1: %4.1f | RSSI2: %4.1f | Act: %d    ", 
               d.x, d.y, d.theta * 57.29, r1, r2, action);
        fflush(stdout);

        // Export en JSON
        exportStateToJSON(&w, "web/state.json");

        // Attendre 10ms (100 FPS)
        usleep(10000);

    }

    setNonBlockingMode(0);
    printf("\nSimulation terminée.\n");
    return 0;
}
