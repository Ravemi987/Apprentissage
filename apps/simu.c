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
    Drone d = { .x = 100, .y = 10, .z = 50, .vx = 0, .vy = 0, .vz = 0, .theta_pitch = 0, .theta_roll = 0 };
    User users[2] = { {40, 0, 50}, {160, 0, 50} }; 
    World w = { .drone = &d, .users = users, .numUsers = 2, .width = 200, .height = 100, .depth = 100 };

    setNonBlockingMode(1);
    printf("Pilotez le drone : [q] Gauche | [d] Droite | [z] Full | [s] Rien | [x] Quitter\n");

    char ch;
    int action = ENGINE_NONE;
    int running = 1;
    int timeout = 0; // Compteur pour relâcher automatiquement la touche

    while (running) {
        // Lecture de la touche
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == 'q') action = ENGINE_LEFT;
            else if (ch == 'd') action = ENGINE_RIGHT;
            else if (ch == 'z') action = ENGINE_FULL;
            else if (ch == 's') action = ENGINE_NONE;
            else if (ch == 'f') action = ENGINE_FORWARD;
            else if (ch == 'b') action = ENGINE_BACKWARD;
            else if (ch == 'x') running = 0;
            timeout = 30; // Maintient l'action pendant 30 frames (300 ms)
        } else {
            // Si aucune touche n'est pressée, on décrémente. À 0, on coupe les moteurs.
            if (timeout > 0) timeout--;
            else action = ENGINE_NONE; 
        }

        // Mise à jour physique
        physicsStep(&w, action, 0.01);

        // Calcul des signaux
        double r1 = computeRSSI(&d, &users[0]);
        double r2 = computeRSSI(&d, &users[1]);

        // Affichage dynamique
        printf("\rPos: (%.1f, %.1f, %.1f) | R: %4.1f° P: %4.1f° | RSSI1: %4.1f | RSSI2: %4.1f | Act: %d    ", 
            d.x, d.y, d.z, d.theta_roll * 57.29, d.theta_pitch * 57.29, r1, r2, action);
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
