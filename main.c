#include "expansion.h"
#include "spanning.h"
#include "balancing.h"

#include <stdio.h>
int numdrones = 0;
int main(int argc, char *argv[])
{
    FILE *fp;
    fp = fopen("output.txt", "w");
    // srand((unsigned int)time(NULL)); // seed should be called one time not in the function that will be called each time
    srand(0);
    if (argc < 2)
    {
        printf("Please provide a number of drone as argument ex.\"n=60\" \n");
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

    // int targets_size = numdrones * 0.20;
    // Target targets[targets_size];
    //  generates coordiantes of targets
    //  generate_random_targets(targets, targets_size);

    const int targets_size = 8; // Since you have 8 specific targets
    Target targets[targets_size];

    // Coordinates of the specific targets
    float predefinedTargets[8][2] = {
        {0, 85}, {0, -85}, {85, 0}, {-85, 0}, {75, 75}, {75, -75}, {-75, 75}, {-75, -75}};

    // // Assign these coordinates to your targets array
    // for (int i = 0; i < targets_size; i++)
    // {
    //     targets[i].x = predefinedTargets[i][0];
    //     targets[i].y = predefinedTargets[i][1];
    // }

    generate_targets(targets, targets_size, predefinedTargets);
    // generate_targets(targets, targets_size, predefinedTargets);

    save_targes(targets, targets_size, fp);

    Drones drones[numdrones];
    initializeDrones(drones, numdrones);

    // for the neigboor
    // create a Neighbors for the neighboor ( one for each drone, so each drone has data of the s1-s6)
    struct Neighbors DroneNeighbors[numdrones];
    for (int i = 0; i < numdrones; i++)
    {
        initNeighbors(&DroneNeighbors[i]);
    }

    /*
    When you declare an array of Drones as Drones drones[numdrones], it creates an array of numdrones elements on the stack.
    These pointers (direction_taken and border_neighbors) are uninitialized .
    Therefore, you will expereience undefined behaviorif passing uninitialized pointers to the append_drones_neighbors_names function, .
    */
    // spread from the sink;
    for (int i = 0; i < numdrones; i++)
    {
        int dir = randomInt(1, 6); // remeber dir are from 0-6 but here number is between 1-6 so no drone start at the sink because that will lead to worng pripority
        // printf(" drone %d goes to %d\n", drones[i].id, dir);
        moveDrones(&drones[i], dir);
    }
    saveDrones(drones, numdrones, fp);

    // init the 6 neighboors distances
    for (int i = 0; i < numdrones; i++)
    {
        creatSpots(&DroneNeighbors[i], drones[i].x, drones[i].y); // drones[i].x, drones[i].y are the coordinates drom (0,0) the sink
    }

    perform_first_expansion(drones, DroneNeighbors, numdrones, fp);
    form_border_and_update_states(drones, DroneNeighbors, numdrones, targets, targets_size, fp);
    // END of expansion phase
    perform_spanning(drones, DroneNeighbors, numdrones, fp);
    // End of spanning phase
    perform_balancing_phase(drones, DroneNeighbors, numdrones, fp);

    for (int i = 0; i < numdrones; i++)
    {
        drones[i].previous_state = drones[i].state;
    }

    int num_drones_border_irrmovable = 0;
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state == Border || drones[i].state == Irremovable || drones[i].state == Irremovable_border)
            num_drones_border_irrmovable++;
    }
    int round = 0;
    while (num_drones_border_irrmovable < numdrones)
    {
        num_drones_border_irrmovable = 0;
        perform_further_expansion(drones, DroneNeighbors, numdrones, fp);
        printf(" \n\n furthere expansion done \n\nn");

        // another finiding border
        form_further_border_and_update_states(drones, DroneNeighbors, numdrones, targets, targets_size, fp);
        printf(" \n\n furthere form_further_border_and_update_states done \n\nn");
        // // another balancing
        perform_further_spanning(drones, DroneNeighbors, numdrones, fp);
        perform_balancing_phase(drones, DroneNeighbors, numdrones, fp);
        printf(" \n\n perform_balancing_phase done \n\nn");

        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].state == Border || drones[i].state == Irremovable || drones[i].state == Irremovable_border)
                num_drones_border_irrmovable++;
        }
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].previous_state != Irremovable)
                drones[i].previous_state = drones[i].state;
        }
    }
    saveDrones(drones, numdrones, fp);

    free_memory_drone(drones, numdrones);
    printf("--------------------------------------------------------------------\n");
    printf(" ----------Drones positions are calculated in all phaes----------\n");
    printf("--------------------------------------------------------------------\n");
    printf("        ---------- PLoting Data in process ----------               \n");

    fclose(fp);
    return 0;
}
