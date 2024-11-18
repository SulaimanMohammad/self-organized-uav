#include "expansion.h"
#include "spanning.h"
#include "balancing.h"
#include <stdio.h>
#define MAX_TARGETS 100

int numdrones = 0;
void parse_predefined_targets(char *line, float predefinedTargets[][2], int size)
{
    // Check if the line starts with '#' and return immediately if it does
    if (line[0] == '#')
    {
        return;
    }
    char *token = strtok(line, "{}, ");
    for (int i = 0; i < size && token != NULL; i++)
    {
        predefinedTargets[i][0] = atof(token); // x-coordinate
        token = strtok(NULL, "{}, ");
        if (token != NULL)
        {
            predefinedTargets[i][1] = atof(token); // y-coordinate
            token = strtok(NULL, "{}, ");
        }
    }
}
void parse_parameters(int *targets_size, bool *random_targets, float predefinedTargets[MAX_TARGETS][2])
{
    // Parse parameters from file
    FILE *params_file = fopen("../benchmark/parameters.txt", "r");
    char line[256];
    while (fgets(line, sizeof(line), params_file))
    {
        if (sscanf(line, "size_of_target=%d", targets_size) == 1)
            continue;
        if (strstr(line, "predfined_targets_random=") != NULL)
        {
            // Parse boolean value for random_targets
            if (strstr(line, "true") != NULL)
            {
                *random_targets = true;
            }
            else if (strstr(line, "false") != NULL)
            {
                *random_targets = false;
            }
            continue;
        }
        if (strstr(line, "predfined_targets=") != NULL)
        {
            char *target_values = strchr(line, '{');
            if (target_values != NULL)
            {
                parse_predefined_targets(target_values, predefinedTargets, *targets_size);
            }
        }
    }
    fclose(params_file);
}

int main(int argc, char *argv[])
{
    FILE *fp;
    fp = fopen("output.txt", "w");
    srand((unsigned int)time(NULL)); // seed should be called one time not in the function that will be called each time
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
                break;
            }
        }
    }

    int targets_size = 0;
    bool random_targets = false;
    float predefinedTargets[MAX_TARGETS][2];
    parse_parameters(&targets_size, &random_targets, predefinedTargets);
    Target targets[targets_size];
    if (random_targets)
        generate_targets(targets, targets_size, NULL);
    else
        generate_targets(targets, targets_size, predefinedTargets);

    save_targes(targets, targets_size, fp);

    Drones drones[numdrones];
    initializeDrones(drones, numdrones);

    // create a Neighbors for the neighboor (one for each drone, so each drone has data of the s1-s6)
    struct Neighbors DroneNeighbors[numdrones];
    for (int i = 0; i < numdrones; i++)
    {
        initNeighbors(&DroneNeighbors[i]);
    }

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
    int number_of_rounds = 1;
    perform_first_expansion(drones, DroneNeighbors, numdrones, fp);
    form_border_and_update_states(drones, DroneNeighbors, numdrones, targets, targets_size, fp);
    perform_spanning(drones, DroneNeighbors, numdrones, fp);
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
    printf("Round %d finished\n", number_of_rounds);
    while (num_drones_border_irrmovable < numdrones)
    {

        num_drones_border_irrmovable = 0;
        perform_further_expansion(drones, DroneNeighbors, numdrones, fp);
        form_further_border_and_update_states(drones, DroneNeighbors, numdrones, targets, targets_size, fp);
        perform_further_spanning(drones, DroneNeighbors, numdrones, fp);
        perform_balancing_phase(drones, DroneNeighbors, numdrones, fp);
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
        number_of_rounds++;
        printf("Round %d finished\n", number_of_rounds);
    }
    saveDrones(drones, numdrones, fp);

    free_memory_drone(drones, numdrones);
    printf("--------------------------------------------------------------------------------------\n");
    printf("    -------- VESPA is done, Drones positions are calculated in all phases --------    \n");
    printf("--------------------------------------------------------------------------------------\n");
    printf("                 ---------- PLoting Data in process ----------               \n");

    fclose(fp);
    return 0;
}
