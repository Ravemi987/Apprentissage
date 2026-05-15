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
    srand(time(NULL));

    // Initialisation des structutres
    Drone d = createDrone(100.0, 50.0, 10.0);
    User users[2] = { {40, 40, 0}, {160, 60, 0} }; 
    World w = { .drone = &d, .users = users, .numUsers = 2, .width = 200, .height = 100, .depth = 100 };

    setNonBlockingMode(1);
    printf("Contrôles : [z/s] Alt | [q/d] Roll | [a/e] Pitch | [r/f] Yaw | [x] Quitter\n");

    char ch;
    int current_action = -1; 
    int running = 1;

    while (running) {
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == 'z') current_action = ENGINE_UP;
            else if (ch == 's') current_action = ENGINE_DOWN;
            else if (ch == 'q') current_action = ENGINE_ROLL_LEFT;
            else if (ch == 'd') current_action = ENGINE_ROLL_RIGHT;
            else if (ch == 'a') current_action = ENGINE_PITCH_LEFT;
            else if (ch == 'e') current_action = ENGINE_PITCH_RIGHT;
            else if (ch == 'r') current_action = ENGINE_YAW_LEFT;
            else if (ch == 'f') current_action = ENGINE_YAW_RIGHT;
            else if (ch == 'w') current_action = ENGINE_IDLE;
            else if (ch == 'x') running = 0; 
        }

        handleCommand(&d, current_action);
 
        physicsStep(&w, DT);

        printf("\rPos: (%.1f, %.1f, %.1f) | R:%.1f° P:%.1f° Y:%.1f°", 
               d.x, d.y, d.z, d.phi*57.3, d.theta*57.3, d.psi*57.3);
        fflush(stdout);

        exportStateToJSON(&w, "web/state.json");

        usleep(10000);
    }

    setNonBlockingMode(0);
    return 0;
}
