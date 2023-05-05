#include "expansion.h"
#include <stdio.h>
int numdrones = 0;
int main(int argc, char *argv[])
{
    FILE *fp;
    fp = fopen("output.txt", "w");
    srand((unsigned int)time(NULL)); // seed should be called one time not in the function that will be called each time

    if (argc < 2)
    {
        printf("Please provide a number as argument\n");
        return 1;
    }
    else
    {
        for (int i = 1; i < argc; i++)
        {
            if (sscanf(argv[i], "n=%d", &numdrones) == 1)
            {
                // Found the argument "n"
                break;
            }
        }
    }
    // for the neigboor
    // create a Neighbors for the neighboor ( one for each drone, so each drone has data of the s1-s6)
    struct Neighbors DroneNeighbors[numdrones];
    for (int i = 0; i < numdrones; i++)
    {
        initNeighbors(&DroneNeighbors[i]);
    }

    Drones drones[numdrones];
    initializeDrones(drones, numdrones);

    // spread from the sink;
    for (int i = 0; i < numdrones; i++)
    {
        int dir = randomInt(1, 6); // remeber dir are from 0-6 but here number is between 1-6 so no drone start at the sink because that will lead to worng pripority
        moveDrones(&drones[i], dir);
    }
    saveDrones(drones, numdrones, fp);

    // init the 6 neighboors distances
    for (int i = 0; i < numdrones; i++)
    {
        creatSpots(&DroneNeighbors[i], drones[i].x, drones[i].y); // drones[i].x, drones[i].y are the coordinates drom (0,0) the sink
    }

    char Priority[MAX_SIZE][MAX_SIZE];
    int PrioritySize;
    int dir;
    for (int k = 0; k < 8; k++)
    {
        // each point now need to check around then move then the second oen do that
        // the ones before should be moved so the next one can detect the new position
        for (int i = 0; i < numdrones; i++)
        {
            // printf("---------point %d at (%f,%f) ---------\n", i, drones[i].x, drones[i].y);
            setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration

            set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
            setPriorities(&DroneNeighbors[i]);

            findPriority(&DroneNeighbors[i], Priority, &PrioritySize); // that should return only one number, since there is also random number generator
                                                                       // if the random numbering is not considered then there will be many possible solution
                                                                       // and that if the spots has the same number of dron in it and using the [f(w,c,eps), f(w,c)[
                                                                       // then same number will be choosed for pts has same drons

            // printf("point %d at (%f,%f) Go to  %s with distance %f and priort %f\n", i, drones[i].x, drones[i].y, Priority[0], getDist(&DroneNeighbors[i], Priority[0]), getPriority(&DroneNeighbors[i], Priority[0]));
            sscanf(Priority[0], "s%d", &dir);
            moveDrones(&drones[i], dir);
            setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration
        }
        saveDrones(drones, numdrones, fp);
    }
    fclose(fp);
    return 0;
}
