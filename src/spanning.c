#include "expansion.h"

int isDuplicate(Target *objects, int size, float x, float y)
{
    for (int i = 0; i < size; i++)
    {
        if (objects[i].x == x && objects[i].y == y)
        {
            return 1;
        }
    }
    return 0;
}

// // generate targets
void generate_random_targets(Target *targets, int targets_size)
{
    int n = targets_size * 0.50; // to detrmine
    int dir1, dir2;
    int mult1, mult2;
    for (int i = 0; i < targets_size; i++)
    {
        float x, y;
        do
        {
            dir1 = rand() % (6 + 1);
            dir2 = rand() % (6 + 1);
            mult1 = (rand() % (n + 1));
            mult2 = (rand() % (n + 1)); // Generate a random value between 0 and n (inclusive)
            x = mult1 * DIR_VECTORS[dir1][0];
            y = mult1 * DIR_VECTORS[dir1][1];

        } while ((abs(x) < 1000 && abs(y) < 1000) && isDuplicate(targets, i, x, y));

        targets[i].x = x;
        targets[i].y = y;
    }
}

void save_targes(Target *targets, int targets_size, FILE *fp)
{
    // Print the generated targets
    for (int i = 0; i < targets_size; i++)
    {

        fprintf(fp, "(");
        fprintf(fp, "%d", 00);
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

void set_state_target_check(Drones drones[], Drones *currentDrones, Target *targets, int targets_size, int numdrones)
{
    // so when drone was border then will move many steps then border state will not be saved
    int previous_state = 0;
    for (int i = 0; i < targets_size; i++)
    {
        if (currentDrones->x == targets[i].x && currentDrones->y == targets[i].y)
        {

            previous_state = currentDrones->state;

            if (currentDrones->state == 0 || currentDrones->state == 1) // drone is in free or alone state
            {
                currentDrones->state = 3; // drone is irrmovable
            }
            if (currentDrones->state == 2) // drone is in border state
            {
                currentDrones->state = 4; // drone is irrmovable and border
                update_irrmovable_border_state(drones, &drones[i], numdrones);
            }
            if (currentDrones->state != previous_state)
                currentDrones->previous_state = previous_state;
        }
    }
}

void findIrremovableDroneAround(Drones drones[], Drones *currentDrone, char result[MAX_SIZE][MAX_SIZE], int *irrmvble_id, int *resultSize, int numdrones)
{
    int count = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrone->x + DIR_VECTORS[j][0] && drones[i].y == currentDrone->y + DIR_VECTORS[j][1] && (drones[i].state == 3 || drones[i].state == 4))
            {
                char neighbors[3];
                sprintf(neighbors, "s%d", j);
                strcpy(result[count], neighbors);
                irrmvble_id[count] = drones[i].id;
                count++;
            }
        }
    }
    *resultSize = count;
}

void find_num_IrremovableDroneAround(Drones drones[], Drones *currentDrone, int *resultSize, int numdrones, int currentDrones_id, int sender_id)
{
    int count = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrone->x + DIR_VECTORS[j][0] &&
                drones[i].y == currentDrone->y + DIR_VECTORS[j][1] &&
                (drones[i].state == 3 || drones[i].state == 4) &&
                drones[i].id != sender_id &&
                drones[i].id != currentDrones_id)
            {
                count++;
            }
        }
    }
    *resultSize = count;
}

int has_irrmovable_drone_around_to_sink(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    int dir = 0;
    int min_distance_irrmovable = 0;
    int id = 0;
    int dir_to_check = 0;
    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrones->x + DIR_VECTORS[j][0] && drones[i].y == currentDrones->y + DIR_VECTORS[j][1])
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    break;
                }
            }
        }
    }

    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrones->x + DIR_VECTORS[j][0] && drones[i].y == currentDrones->y + DIR_VECTORS[j][1])
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0 && neighbors[drones[i].id].distances[0] <= min_distance_irrmovable)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    dir = j;
                }
            }
        }
    }
    return dir;
}

int has_irrmovable_drone_around_to_border(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    int dir = 0;
    int min_distance_irrmovable = 0;
    int id = 0;
    int dir_to_check = 0;
    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrones->x + DIR_VECTORS[j][0] && drones[i].y == currentDrones->y + DIR_VECTORS[j][1])
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    break;
                }
            }
        }
    }

    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrones->x + DIR_VECTORS[j][0] && drones[i].y == currentDrones->y + DIR_VECTORS[j][1])
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0 && neighbors[drones[i].id].distances[0] >= min_distance_irrmovable)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    dir = j;
                }
            }
        }
    }
    return dir;
}

void findDirofSender(Drones drones[], Drones *currentDrone, char sender_dir[3], int numdrones, int sender_id)
{

    int count = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrone->x + DIR_VECTORS[j][0] && drones[i].y == currentDrone->y + DIR_VECTORS[j][1] && (drones[i].id == sender_id))
            {
                sprintf(sender_dir, "s%d", j);
                break;
            }
        }
    }
}

int check_close_to_sink(Drones drones[], Drones *currentDrones, int numdrones, int dir)
{
    int closetosink_drone = 0;
    for (int i = 0; i < numdrones; i++)
    {
        // find what drone it is
        if (drones[i].x == currentDrones->x + DIR_VECTORS[dir][0] && drones[i].y == currentDrones->y + DIR_VECTORS[dir][1])
        {
            if (abs(drones[i].x) < abs(currentDrones->x) && abs(drones[i].y) < abs(currentDrones->y))
            {
                closetosink_drone = 1;
            }
            break;
        }
    }
    return closetosink_drone;
}

int check_previous_border(Drones drones[], Drones *currentDrones, int numdrones, int dir)
{
    int previous_border = 0;
    for (int i = 0; i < numdrones; i++)
    {
        // find what drone it is
        if (drones[i].x == currentDrones->x + DIR_VECTORS[dir][0] && drones[i].y == currentDrones->y + DIR_VECTORS[dir][1])
        {
            if ((drones[i].previous_state == 2 || drones[i].previous_state == 4)) // it was a border before
            {
                previous_border = 1;
                break;
            }
        }
    }
    return previous_border;
}

void build_path_to_sink(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    // the drone that found object and start finding the path to the sink
    int distant_of_object = 0;
    //

    if (currentDrones->id == sender_id)
    {
        distant_of_object = neighbors[currentDrones->id].distances[0];
    }

    if ((currentDrones->x == 0 && currentDrones->y == 0) || (currentDrones->previous_state == 3)) // stop the recursion when arrive to sink
    {
        return;
    }

    int dir = 0;
    char irrmvble_dir[MAX_SIZE][MAX_SIZE];
    int irrmvble_id[6]; // all id  of irrmovable
    int irrmvbleSize = 0;
    setDist(&neighbors[currentDrones->id], currentDrones->x, currentDrones->y); // update for the next iteration                                                        // find the spots closer to the sink to decide how to set the priorities
    char closeSink[MAX_SIZE][MAX_SIZE];
    int closeSinkSize = 0;
    findMinDistances(&neighbors[currentDrones->id], closeSink, &closeSinkSize);
    findIrremovableDroneAround(drones, currentDrones, irrmvble_dir, irrmvble_id, &irrmvbleSize, numdrones); // it will accept only the one close to the sink
    //   find the direction of the drone that send message to build the path
    char sender_dir[3];
    findDirofSender(drones, currentDrones, sender_dir, numdrones, sender_id);

    for (int k = 0; k < irrmvbleSize; k++)
    {
        if (irrmvble_id[k] != sender_id && neighbors[currentDrones->id].distances[0] > neighbors[irrmvble_id[k]].distances[0]) // getting closer to sink
        {
            sscanf(irrmvble_dir[k], "s%d", &dir);
            break;
        }
    }

    if (dir == 0) // here you need to find spot occupied and close to the sink as possible
    {
        set_num_drones_at_neighbors(drones, &neighbors[currentDrones->id], currentDrones, numdrones);
        setDist(&neighbors[currentDrones->id], currentDrones->x, currentDrones->y); // update for the next iteration                                                        // find the spots closer to the sink to decide how to set the priorities
        float min_distnace;

        // set the first value as the distance of spot from sink and the spot should be occupied so we can compare
        for (int j = 1; j < neighbors[currentDrones->id].size; j++) // s0 is not included
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0) // currentDrones->previous_state == 2
            {
                min_distnace = neighbors[currentDrones->id].distances[j];
                dir = j;
                break;
            }
        }

        for (int j = 1; j < neighbors[currentDrones->id].size; j++) // neighbors[currentDrones->id].size=7
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                neighbors[currentDrones->id].distances[j] <= min_distnace)
            {
                // do not conisder that if it was the sender
                if (strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0) // that prevent to make it come back
                {
                    min_distnace = neighbors[currentDrones->id].distances[j];
                    dir = j;
                }
            }
        }
    }
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].x == currentDrones->x + DIR_VECTORS[dir][0] && drones[i].y == currentDrones->y + DIR_VECTORS[dir][1])
        {
            if (drones[i].state == 2) // it is border then make it border and irrmovable
            {
                drones[i].state = 4;
                currentDrones->previous_state = 3;
                // you should go to the sink
                return build_path_to_sink(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
            else if (drones[i].state == 3 || drones[i].state == 4)
            {
                return; // arrived to drone with irrmovable state
            }
            else
            {
                // printf(" go in else \n");
                drones[i].state = 3;
                currentDrones->previous_state = 3;
                return build_path_to_sink(neighbors, &drones[i], drones, numdrones, currentDrones->id);
                // saveDrones(drones, numdrones, fp);
            }
        }
    }
}

void build_path_to_sink_further(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{

    set_num_drones_at_neighbors(drones, &neighbors[currentDrones->id], currentDrones, numdrones);

    // the drone that found object and start finding the path to the sink
    int distant_of_object = 0;
    if (currentDrones->id == sender_id)
    {
        distant_of_object = neighbors[currentDrones->id].distances[0];
    }

    if ((currentDrones->x == 0 && currentDrones->y == 0) || (currentDrones->previous_state == 3)) // stop the recursion when arrive to sink
    {
        return;
    }

    int dir = 0;
    char irrmvble_dir[MAX_SIZE][MAX_SIZE];
    int irrmvble_id[6]; // all id  of irrmovable
    int irrmvbleSize = 0;
    setDist(&neighbors[currentDrones->id], currentDrones->x, currentDrones->y); // update for the next iteration                                                        // find the spots closer to the sink to decide how to set the priorities
    char closeSink[MAX_SIZE][MAX_SIZE];
    int closeSinkSize = 0;
    findMinDistances(&neighbors[currentDrones->id], closeSink, &closeSinkSize);
    findIrremovableDroneAround(drones, currentDrones, irrmvble_dir, irrmvble_id, &irrmvbleSize, numdrones); // it will accept only the one close to the sink
    //      find the direction of the drone that send message to build the path
    char sender_dir[3];
    findDirofSender(drones, currentDrones, sender_dir, numdrones, sender_id);
    float min_irrmovable;
    if (irrmvbleSize > 0)
    {
        for (int k = 0; k < irrmvbleSize; k++)
        {
            if (irrmvble_id[k] != sender_id && neighbors[currentDrones->id].distances[0] > neighbors[irrmvble_id[k]].distances[0])
            {
                min_irrmovable = neighbors[irrmvble_id[k]].distances[0];
                sscanf(irrmvble_dir[k], "s%d", &dir);
                break;
            }
        }
    }
    for (int k = 0; k < irrmvbleSize; k++)
    {
        if (irrmvble_id[k] != sender_id &&
            neighbors[currentDrones->id].distances[0] > neighbors[irrmvble_id[k]].distances[0] &&
            neighbors[irrmvble_id[k]].distances[0] < min_irrmovable && check_close_to_sink(drones, currentDrones, numdrones, k)) // getting closer to sink
        {
            min_irrmovable = neighbors[irrmvble_id[k]].distances[0];
            sscanf(irrmvble_dir[k], "s%d", &dir);
        }
    }

    if (dir == 0)
    {
        for (int j = 1; j < neighbors[currentDrones->id].size; j++) // neighbors[currentDrones->id].size=7
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0)
            {
                if (check_previous_border(drones, currentDrones, numdrones, j)) //||has_irrmovable_drone_around(neighbors, currentDrones, drones, numdrones, sender_id)) // you need to passs J
                {
                    dir = j;
                    break;
                }
            }
        }
    }

    // go to a drone that has an irrmovale drone
    if (dir == 0)
    {
        dir = has_irrmovable_drone_around_to_sink(neighbors, currentDrones, drones, numdrones, sender_id);
    }

    if (dir == 0)
    {
        for (int j = 1; j < neighbors[currentDrones->id].size; j++) // neighbors[currentDrones->id].size=7
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0)
            {
                int border = 0;
                for (int i = 0; i < numdrones; i++)
                {
                    // find what drone it is
                    if (drones[i].x == currentDrones->x + DIR_VECTORS[j][0] && drones[i].y == currentDrones->y + DIR_VECTORS[j][1])
                    {
                        if ((drones[i].state == 2 || drones[i].state == 4) && (drones[i].previous_state == 2 || drones[i].previous_state == 4))
                        {
                            border = 1;
                            break;
                        }
                    }
                }
                if (border)
                {
                    dir = j;
                    break;
                }
            }
        }
    }

    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].x == currentDrones->x + DIR_VECTORS[dir][0] && drones[i].y == currentDrones->y + DIR_VECTORS[dir][1])
        {
            if (drones[i].state == 2) // it is border then make it border and irrmovable
            {
                drones[i].state = 4;
                currentDrones->previous_state = 3;
                // you should go to the sink
                return build_path_to_sink_further(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
            else if (drones[i].state == 3 || drones[i].state == 4 && (drones[i].previous_state == 2 || drones[i].previous_state == 4))
            {
                return;
            }
            else
            {
                drones[i].state = 3;
                currentDrones->previous_state = 3; // this is to avoid considrringg what already done as a new target
                                                   // see what happen without it // and no need to try to conider the road each iteration
                return build_path_to_sink_further(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
        }
    }
}

/*
here you should connect to the farest and close to the border or irrmovable
*/
// this should be doen until arriving to a drone with a state border
void build_path_to_border(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    // printf("curre_drone %d \n", currentDrones->id);
    set_num_drones_at_neighbors(drones, &neighbors[currentDrones->id], currentDrones, numdrones);
    // no need to do so if the drone is irrmovable and border
    if (currentDrones->state == 4 || currentDrones->state == 2) //|| currentDrones->previous_state == 3) // stop the recursion when arrive to border
    {
        return;
    }
    int dir = 0;
    char irrmvble_dir[MAX_SIZE][MAX_SIZE];
    int irrmvble_id[6]; // all id  of irrmovable
    int irrmvbleSize = 0;
    // it will accept only the one close to the sink
    setDist(&neighbors[currentDrones->id], currentDrones->x, currentDrones->y); // find the spots distances

    char closeBorder[MAX_SIZE][MAX_SIZE];
    int closeBorderSize;
    findMaxDistances(&neighbors[currentDrones->id], closeBorder, &closeBorderSize); // find spot that is far from the sink ( towards the border)
    char sender_dir[3];
    findIrremovableDroneAround(drones, currentDrones, irrmvble_dir, irrmvble_id, &irrmvbleSize, numdrones); // it will accept only the one close to the sink
    //    find the direction of the drone that send message to build the path
    findDirofSender(drones, currentDrones, sender_dir, numdrones, sender_id);
    // if (irrmvbleSize > 0)
    // {
    for (int k = 0; k < irrmvbleSize; k++)
    {
        // printf("  irrmvble_id [%d]= %d in distance %f\n", k, irrmvble_id[k], neighbors[irrmvble_id[k]].distances[0]);
        if (irrmvble_id[k] != sender_id && neighbors[currentDrones->id].distances[0] < neighbors[irrmvble_id[k]].distances[0])
        {
            // printf("  Chosen [%d]= %d\n", k, irrmvble_id[k]);

            sscanf(irrmvble_dir[k], "s%d", &dir);
            break;
        }
    }

    // no need to go do s same as the go to the sink because going to the border is easy to go to the border
    if (dir == 0)
    {
        int max_dist = 0;
        for (int j = 1; j < neighbors[currentDrones->id].size; j++) // neighbors[currentDrones->id].size=7
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0 &&
                neighbors[currentDrones->id].distances[0] < neighbors[currentDrones->id].distances[j])
            {

                max_dist = neighbors[currentDrones->id].distances[j];
                break;
            }
        }

        for (int j = 1; j < neighbors[currentDrones->id].size; j++) // neighbors[currentDrones->id].size=7
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0 &&
                max_dist < neighbors[currentDrones->id].distances[j])
            {

                max_dist = neighbors[currentDrones->id].distances[j];
                dir = j;
            }
        }
    }

    // if (dir == 0)
    // {
    //     dir = has_irrmovable_drone_around_to_border(neighbors, currentDrones, drones, numdrones, sender_id);
    // }
    //  same in real life after reciving a message should check the state of the drone if it is not border it should forward the message
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].x == currentDrones->x + DIR_VECTORS[dir][0] && drones[i].y == currentDrones->y + DIR_VECTORS[dir][1])
        {
            if (drones[i].state == 2) // it is border then make it border and irrmovable
            {
                drones[i].state = 4;
                currentDrones->previous_state = 3;
                return; // build_path_to_border(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
            else if (drones[i].state == 3 || drones[i].state == 4)
            {
                return; // arrived to drone with irrmovable state
            }
            else
            {
                drones[i].state = 3;
                currentDrones->previous_state = 3;
                return build_path_to_border(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
        }
    }
    // }
}

void perform_spanning(Drones drones[], struct Neighbors DroneNeighbors[], int numdrones, FILE *fp)
{
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state == 3 || drones[i].state == 4)
        {
            set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
            build_path_to_sink(DroneNeighbors, &drones[i], drones, numdrones, drones[i].id);
        }
    }
    // saveDrones(drones, numdrones, fp);
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state == 3 || drones[i].state == 4)
        {
            set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
            build_path_to_border(DroneNeighbors, &drones[i], drones, numdrones, drones[i].id);
        }
    }
    saveDrones(drones, numdrones, fp);
}

void perform_further_spanning(Drones drones[], struct Neighbors DroneNeighbors[], int numdrones, FILE *fp)
{
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state == 3 || drones[i].state == 4)
        {
            set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
            build_path_to_sink_further(DroneNeighbors, &drones[i], drones, numdrones, drones[i].id);
        }
    }

    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state == 3 || drones[i].state == 4)
        {
            set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
            build_path_to_border(DroneNeighbors, &drones[i], drones, numdrones, drones[i].id);
        }
    }
    saveDrones(drones, numdrones, fp);
}