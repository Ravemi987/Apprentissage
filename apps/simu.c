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

    // 2. Définition de la population (Utilisateurs fixes pour le test)
    User users[] = { 
        {170.0, 80.0, 0.0}, 
        {165.0, 75.0, 0.0},
        {175.0, 85.0, 0.0},
        {180.0, 70.0, 0.0}
    }; 
    int num_users = sizeof(users) / sizeof(users[0]);

    // 3. Définition des obstacles 3D
    Obstacle3D obstacles[] = {
        {30.0, 20.0, 0.0, 3.0, 15.0},   
        {30.0, 80.0, 0.0, 4.0, 20.0},  
        {100.0, 10.0, 0.0, 2.5, 12.0}
    };
    int num_obstacles = sizeof(obstacles) / sizeof(obstacles[0]);

    // Assemblage du monde 3D
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
