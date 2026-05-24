#include <simulation.h>
#include <generation.h>
#include <unistd.h>


int main() {
    Drone drone = createDrone(100.0, 50.0, 10.0);
    int seed = 1234;
    float width = 200.0;
    float height = 200.0;
    float depth = 100.0;
    int numUser = 5;
    int numObstacle = 5;
    World w = creationWorld(&drone, numUser, numObstacle, width, height, depth, -1);
    for (int i=0;i<10;i++) {
        majWorld(&w, TOTAL_RAND);
        exportStateToJSON(&w, "web/state.json");
        sleep(2);

    }

    free(w.users);
    free(w.obstacles);
}