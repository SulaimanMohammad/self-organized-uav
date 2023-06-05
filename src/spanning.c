#include "expansion.h"

// generate targets
void generate_random_targets(Target *targets, int targets_size)
{
    int n = 5; // to detrmine
    int dir, mult;
    for (int i = 0; i < targets_size; i++)
    {

        // targets[i][j] = rand() % (n + 1); // Generate a random value between 0 and n (inclusive)
        dir = rand() % (6 + 1);
        mult = (rand() % (n + 1));
        targets[i].x = mult * DIR_VECTORS[dir][0];
        targets[i].y = mult * DIR_VECTORS[dir][1];
    }
}
void save_targes(Target *targets, int targets_size, FILE *fp)
{
    // Print the generated targets
    for (int i = 0; i < targets_size; i++)
    {

        fprintf(fp, "(");
        fprintf(fp, "%d", 1111);
        fprintf(fp, ", ");
        fprintf(fp, "%.6f", targets[i].x);
        fprintf(fp, ", ");
        fprintf(fp, "%.6f", targets[i].y);
        fprintf(fp, ", ");
        fprintf(fp, "%d", 10);
        fprintf(fp, "),");
    }
    fprintf(fp, "\n");
}

void set_state_target_check(Drones *currentDrones, Target *targets, int targets_size)
{
    for (int i = 0; i < targets_size; i++)
    {
        if (currentDrones->x == targets[i].x && currentDrones->y == targets[i].y)
        {
            if (currentDrones->state == 0 || currentDrones->state == 1) // drone is in free or alone state
                currentDrones->state = 3;                               // drone is irrmovable
            if (currentDrones->state == 2)                              // drone is in border state
                currentDrones->state = 4;                               // drone is irrmovable and border
        }
    }
}

void build_path_to_sink(struct Neighbors *neighbors, Drones *currentDrones, Drones drones[], int numdrones)
{
    if (currentDrones->x == 0 && currentDrones->y == 0) // stop the recursion when arrive to sink
    {
        return;
    }

    if (currentDrones->state == 3 || currentDrones->state == 4) // drone is irrmovable (target) if the drone is border still need to be connected to sink
    {
        setDist(neighbors, currentDrones->x, currentDrones->y); // update for the next iteration
                                                                // find the spots closer to the sink to decide how to set the priorities
        char closeSink[MAX_SIZE][MAX_SIZE];
        int closeSinkSize;
        findMinDistances(neighbors, closeSink, &closeSinkSize);
        int dir = 0;
        sscanf(closeSink[0], "s%d", &dir);
        // printf(" dir %d", dir);
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrones->x + DIR_VECTORS[dir][0] && drones[i].y == currentDrones->y + DIR_VECTORS[dir][1])
            {
                if (drones[i].state == 2) // it is border then make it border and irrmovable
                    drones[i].state = 4;
                else
                    drones[i].state = 3;
                return build_path_to_sink(neighbors, &drones[i], drones, numdrones);
            }
        }
    }
}

// this should be doen until arriving to a drone with a state border
void build_path_to_border(struct Neighbors *neighbors, Drones *currentDrones, Drones drones[], int numdrones)
{
    // no need to do so if the drone is irrmovable and border
    if (currentDrones->state == 2 || currentDrones->state == 4) // stop the recursion when arrive to border
    {
        return;
    }

    if (currentDrones->state == 3) // drone is irrmovable (target) //|| currentDrones->state == 4 no need for it because it is already border
    {
        setDist(neighbors, currentDrones->x, currentDrones->y); // find the spots distances
        char closeBorder[MAX_SIZE][MAX_SIZE];
        int closeBorderSize;
        findMaxDistances(neighbors, closeBorder, &closeBorderSize); // find spot that is far from the sink ( towards the border)
        int dir = 0;
        sscanf(closeBorder[0], "s%d", &dir);
        // same in real life after reciving a message should check the state of the drone if it is not border it should forward the message
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrones->x + DIR_VECTORS[dir][0] && drones[i].y == currentDrones->y + DIR_VECTORS[dir][1])
            {
                if (drones[i].state == 2 || drones[i].state == 4) // it is border then make it border and irrmovable
                    drones[i].state = 4;
                else
                    drones[i].state = 3;
                return build_path_to_border(neighbors, &drones[i], drones, numdrones);
            }
        }
    }
}