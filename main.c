#include "expansion.h"
#include <stdio.h>
int numdrones = 0;
int main(int argc, char *argv[])
{
    FILE *fp;
    fp = fopen("output.txt", "w");
    // srand((unsigned int)time(NULL)); // seed should be called one time not in the function that will be called each time

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
        append_new_step(&drones[i], dir);
        // printf("drone %d , went to s%d\n", i, dir);
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
    int num_drone_alone = 0;
    int steps = 0;
    while (num_drone_alone < numdrones)
    {
        // each point now need to check around then move then the second oen do that
        // the ones before should be moved so the next one can detect the new position
        num_drone_alone = 0;
        for (int i = 0; i < numdrones; i++)
        {
            check_drone_spot(drones, &drones[i], numdrones); // check if the drone is alone or not
            // printf("---------drone %d at (%f,%f) has state %d---------\n", i, drones[i].x, drones[i].y, drones[i].state);
            // printf("---------drone %d --------\n", i);

            if (drones[i].state == 1) // free to move and there are many drone in the same place
            {
                setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration

                set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
                setPriorities(&DroneNeighbors[i]);

                findPriority(&DroneNeighbors[i], Priority, &PrioritySize); // that should return only one number, since there is also random number generator
                                                                           // if the random numbering is not considered then there will be many possible solution
                                                                           // and that if the spots has the same number of dron in it and using the [f(w,c,eps), f(w,c)[
                                                                           // then same number will be choosed for pts has same drons

                sscanf(Priority[0], "s%d", &dir);
                moveDrones(&drones[i], dir);
                append_new_step(&drones[i], dir);
                // printf("dones go to %d\n ", dir);
                setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration
                // steps++;
            }
            else // if the drone is alone do not move
            {
                num_drone_alone++;
            }
        }
        saveDrones(drones, numdrones, fp);
    }
    // printf("%d", steps);
    //  After the Drones spreat now it is time to see if it is free or no
    for (int i = 0; i < numdrones; i++)
    {
        // printf("\n--- drone %d-----\n", i);
        update_drone_state(drones, &DroneNeighbors[i], &drones[i], numdrones);
        // countElementOccurrences(&drones[i]);
        free(drones[i].direction_taken); // no need for it any more
    }
    // for (int i = 0; i < numdrones; i++)
    // {

    //     printf("\n--- drone %d----- at state %d \n", i, drones[i].state);
    // }
    fclose(fp);
    return 0;
}
