#include "expansion.h"
#include "spanning.h" // needed for target_check in the expansion

// Define the unit vectors for each direction
// which is the 6 negihboors with respect of the drone position
float DIR_VECTORS[7][2] = {
    {0, 0},                                         // s0 // dont move stay
    {a, 0},                                         // s1
    {(1.0 / 2.0) * a, (3.0 / 2.0) * effective_a},   // s2
    {-(1.0 / 2.0) * a, (3.0 / 2.0) * effective_a},  // s3
    {-1 * a, 0},                                    // s4
    {-(1.0 / 2.0) * a, -(3.0 / 2.0) * effective_a}, // s5
    {(1.0 / 2.0) * a, -(3.0 / 2.0) * effective_a}   // s6

};
// generate random number [A,B[ , b is not included
float randomFloat(float A, float B)
{
    float r = (float)rand() / RAND_MAX;
    return A + r * (B - A);
}

int randomInt(int A, int B)
{
    return (rand() % (B - A + 1) + A);
}

int float_compare(float num1, float num2)
{
    float epsilon = 0.01f; // precision level you need

    if (fabs(num1 - num2) < epsilon)
    {
        return 1; // They are equal
    }
    else
    {
        return 0;
    }
}

void initializeDrones(Drones drones[], int numdrones)
{
    // Initialize all drones at the center of the grid
    for (int i = 0; i < numdrones; i++)
    {
        drones[i].id = i;
        drones[i].x = 0.0;
        drones[i].y = 0.0;
        drones[i].drone_distance = 0.0;
        drones[i].state = Free;
        drones[i].targetfound = 0;
        drones[i].previous_state = Free;
        drones[i].allowed_neighborsSize = 0;
        drones[i].allowed_to_goto = (int *)malloc(MAX_SIZE * sizeof(int));
        for (int j = 0; j < 7; j++)
        {
            drones[i].allowed_to_goto[j] = j;
        }
        drones[i].allowed_neighborsSize = 7;

        drones[i].id_tag_to_border = -1;
        drones[i].id_border_connection = -1;
        drones[i].id_tag_to_sink = -1;
        drones[i].closest_target = -1;
        drones[i].connect_sink = false;
    }
}

void free_memory_drone(Drones drones[], int numdrones)
{
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].allowed_to_goto != NULL)
        {
            free(drones[i].allowed_to_goto);
            drones[i].allowed_to_goto = NULL; // Set to NULL after freeing
        }
    }
}

void printDrones(Drones drones[], int numdrones)
{
    for (int i = 0; i < numdrones; i++)
    {
        printf("(%f, %f), ", drones[i].x, drones[i].y);
    }
    printf("\n");
}

void saveDrones(Drones drones[], int numdrones, FILE *fp)
{
    for (int i = 0; i < numdrones; i++)
    {
        fprintf(fp, "(");
        fprintf(fp, "%d", i);
        fprintf(fp, ", ");
        fprintf(fp, "%.2f", drones[i].x);
        fprintf(fp, ", ");
        fprintf(fp, "%.2f", drones[i].y);
        fprintf(fp, ", ");
        fprintf(fp, "%d", drones[i].state);
        fprintf(fp, ", ");
        fprintf(fp, "%d", drones[i].targetfound);
        fprintf(fp, "),");
    }
    fprintf(fp, "\n");
}

// Initialize the Neighbors
void initNeighbors(struct Neighbors *neighbors)
{
    neighbors->size = 0;
}

// Add a new key-value pair to the Neighbors
void addEntry(struct Neighbors *neighbors, char key[], float value)
{
    if (neighbors->size >= MAX_SIZE)
    {
        printf("Error: Neighbors is full\n");
        return;
    }
    strcpy(neighbors->keys[neighbors->size], key);
    neighbors->distances[neighbors->size] = value;
    neighbors->size++;
}

float round_to_decimals_float(float value, int decimals)
{
    float scale = powf(10.0f, decimals);
    return roundf(value * scale) / scale;
}

// this create the 6 spots with coordiantes regaring the sink not with the respect of the drone position
void creatSpots(struct Neighbors *neighbors, float Dx, float Dy)
{

    float DxDy2 = (Dx * Dx) + (Dy * Dy);
    float DxDy3a2 = DxDy2 + (3 * a * a);
    float sqDx = sqrt3 * Dx;
    float aDx = (2 * sqrt3) * Dx;

    addEntry(neighbors, "s0", round_to_decimals_float(sqrtf(DxDy2), 2));
    addEntry(neighbors, "s1", round_to_decimals_float(sqrtf(DxDy3a2 + a * aDx), 2));
    addEntry(neighbors, "s2", round_to_decimals_float(sqrtf(DxDy3a2 + a * (sqDx + (3 * Dy))), 2));
    addEntry(neighbors, "s3", round_to_decimals_float(sqrtf(DxDy3a2 + a * (3 * Dy - sqDx)), 2));
    addEntry(neighbors, "s4", round_to_decimals_float(sqrtf(DxDy3a2 - aDx * a), 2));
    addEntry(neighbors, "s5", round_to_decimals_float(sqrtf(DxDy3a2 - a * (sqDx + (3 * Dy))), 2));
    addEntry(neighbors, "s6", round_to_decimals_float(sqrtf(DxDy3a2 - a * (3 * Dy - sqDx)), 2));
}

void setDist(struct Neighbors *neighbors, float Dx, float Dy)
{

    float DxDy2 = (Dx * Dx) + (Dy * Dy);
    float DxDy3a2 = DxDy2 + (3 * a * a);
    float sqDx = sqrt3 * Dx;
    float aDx = (2 * sqrt3) * Dx;

    for (int i = 0; i < neighbors->size; i++)
    {
        if (strcmp(neighbors->keys[i], "s0") == 0)
            neighbors->distances[i] = round_to_decimals_float(sqrtf(DxDy2), 2);
        if (strcmp(neighbors->keys[i], "s1") == 0)
            neighbors->distances[i] = round_to_decimals_float(sqrtf(DxDy3a2 + a * aDx), 2);
        if (strcmp(neighbors->keys[i], "s2") == 0)
            neighbors->distances[i] = round_to_decimals_float(sqrtf(DxDy3a2 + a * (sqDx + (3 * Dy))), 2);
        if (strcmp(neighbors->keys[i], "s3") == 0)
            neighbors->distances[i] = round_to_decimals_float(sqrtf(DxDy3a2 + a * (3 * Dy - sqDx)), 2);
        if (strcmp(neighbors->keys[i], "s4") == 0)
            neighbors->distances[i] = round_to_decimals_float(sqrtf(DxDy3a2 - aDx * a), 2);
        if (strcmp(neighbors->keys[i], "s5") == 0)
            neighbors->distances[i] = round_to_decimals_float(sqrtf(DxDy3a2 - a * (sqDx + (3 * Dy))), 2);
        if (strcmp(neighbors->keys[i], "s6") == 0)
            neighbors->distances[i] = round_to_decimals_float(sqrtf(DxDy3a2 - a * (3 * Dy - sqDx)), 2);
        // printf("    %s at distance %f\n ", neighbors->keys[i], neighbors->distances[i]);
    }
}

void setStatus(struct Neighbors *neighbors, char key[], char state[], int W)
{
    // Search for the key in the Neighbors
    for (int i = 0; i < neighbors->size; i++)
    {
        if (strcmp(neighbors->keys[i], key) == 0)
        {
            // Key already exists, update the value
            strcpy(neighbors->Status[i], state);
            neighbors->spot_num_drones[i] = W;
            return;
        }
    }
}

// Get the value associated with a key in the Neighbors
float getDist(struct Neighbors *neighbors, char key[])
{
    for (int i = 0; i < neighbors->size; i++)
    {
        if (strcmp(neighbors->keys[i], key) == 0)
        {
            return neighbors->distances[i];
        }
    }
    printf("Error: Key not found\n");
    return -1;
}

float getPriority(struct Neighbors *neighbors, char key[])
{
    for (int i = 0; i < neighbors->size; i++)
    {
        if (strcmp(neighbors->keys[i], key) == 0)
        {
            return neighbors->spot_priority[i];
        }
    }
    printf("Error: Key not found\n");
    return -1;
}

char *getStatus(struct Neighbors *neighbors, char key[])
{
    for (int i = 0; i < neighbors->size; i++)
    {
        if (strcmp(neighbors->keys[i], key) == 0)
        {
            return neighbors->Status[i];
        }
    }
    printf("Error: Key not found\n");
    return "\0"; // Return a null character to indicate an error;
}

// return the key of object in the minimum distance
void findMinDistances(struct Neighbors *neighbors, char result[MAX_SIZE][MAX_SIZE], int *resultSize)
{
    float minDistance = -1.0;
    int count = 0;

    // Iterate over all entries in the Neighbors
    for (int i = 0; i < neighbors->size; i++)
    {
        // If the distance is positive and less than the current minimum, update the minimum and reset the result array
        if (neighbors->distances[i] > 0 && (minDistance < 0 || neighbors->distances[i] < minDistance))
        {
            minDistance = neighbors->distances[i];
            count = 0;
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
        // If the distance is equal to the current minimum, add the key to the result array
        else if (neighbors->distances[i] == minDistance)
        {
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
    }

    *resultSize = count;
}

// return the key of object in the max distance
void findMaxDistances(struct Neighbors *neighbors, char result[MAX_SIZE][MAX_SIZE], int *resultSize)
{
    float maxDistance = -1.0;
    int count = 0;

    // Iterate over all entries in the Neighbors
    for (int i = 0; i < neighbors->size; i++)
    {

        // If the distance is positive and less than the current minimum, update the minimum and reset the result array
        if (neighbors->distances[i] > 0 && neighbors->distances[i] > maxDistance)
        {
            maxDistance = neighbors->distances[i];
            count = 0;
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
        // If the distance is equal to the current minimum, add the key to the result array
        else if (neighbors->distances[i] == maxDistance)
        {
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
    }

    *resultSize = count;
}

void findPriority(struct Neighbors *neighbors, char result[MAX_SIZE][MAX_SIZE], int *resultSize)
{
    float minDistance = -1.0;
    int count = 0;

    // Iterate over all entries in the Neighbors
    for (int i = 0; i < neighbors->size; i++)
    {
        // If the distance is positive and less than the current minimum, update the minimum and reset the result array
        if (neighbors->spot_priority[i] >= 0 && (minDistance < 0 || neighbors->spot_priority[i] < minDistance))
        {
            minDistance = neighbors->spot_priority[i];
            count = 0;
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
        // If the distance is equal to the current minimum, add the key to the result array
        else if (neighbors->spot_priority[i] == minDistance)
        {
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
    }

    *resultSize = count;
}

void setPriorities(struct Neighbors *neighbors)
{
    // find the spots closer to the sink to decide how to set the priorities
    char closeSink[MAX_SIZE][MAX_SIZE];
    int closeSinkSize;

    findMinDistances(neighbors, closeSink, &closeSinkSize);

    // drone now will set the value of vi that is assiged to each niegboor
    // denom // distance between the place of the drone in position and the sink and it will be 0 if the drone is at the sink
    float denom = (4.0 * getDist(neighbors, "s0"));
    for (int i = 0; i < neighbors->size; i++)
    {
        // if the spot is free
        if (strcmp(neighbors->Status[i], "f") == 0)
        {
            neighbors->spot_priority[i] = neighbors->distances[i] * C / denom;
        }
        else // the spot is occupied
        {
            int isclose = 0;
            // check if the spot is close to the sink
            for (int j = 0; j < closeSinkSize; j++)
            {
                if (strcmp(neighbors->keys[i], closeSink[j]) == 0) // the spot is close to sink
                {
                    isclose = 1;
                    break;
                }
            }
            if (isclose) // it is close to the sink should it should  be avoided
            {
                neighbors->spot_priority[i] = -1; // it is lik INF so it will not be consider when you find the minimum
            }
            else // spot occupied but far from the sink
            {
                neighbors->spot_priority[i] = randomFloat((neighbors->spot_num_drones[i] * C + eps), (neighbors->spot_num_drones[i] + 1) * C); // spot is away  from the sink
            }
        }
    }
}

void findBorderDroneAround(Drones drones[], Drones *currentDrone, char result[MAX_SIZE][MAX_SIZE], int *resultSize, int numdrones)
{

    int count = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
    {
        for (int i = 0; i < numdrones; i++)
        {

            if (float_compare(drones[i].x, currentDrone->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrone->y + DIR_VECTORS[j][1]) && (drones[i].state == Border || drones[i].state == Irremovable || drones[i].state == Irremovable_border))
            {
                char neighbors[3];
                sprintf(neighbors, "s%d", j);
                strcpy(result[count], neighbors);
                count++;
            }
        }
    }
    *resultSize = count;
}

void setPriorities_further_expan(Drones drones[], Drones *currentDrone, struct Neighbors *neighbors, int numdrones)
{
    // find the spots closer to the sink to decide how to set the priorities
    char closeSink[MAX_SIZE][MAX_SIZE];
    int closeSinkSize;
    char borderNeigboor[MAX_SIZE][MAX_SIZE];
    int borderNeigboorSize;

    findMinDistances(neighbors, closeSink, &closeSinkSize);
    findBorderDroneAround(drones, currentDrone, borderNeigboor, &borderNeigboorSize, numdrones);

    int count = 0;
    // drone now will set the value of vi that is assiged to each niegboor
    // denom // distance between the place of the drone in position and the sink and it will be 0 if the drone is at the sink
    float denom = (4.0 * getDist(neighbors, "s0"));
    char intAsString[10]; // Buffer to hold the converted string

    for (int i = 0; i < neighbors->size; i++) // the three allowed spots
    {
        count = 0;
        // for each neighbors check if it is part from the allowed to go
        for (int j = 0; j < currentDrone->allowed_neighborsSize; j++)
        {
            if (currentDrone->allowed_to_goto[j] == 0)
            {
                continue;
            }
            if (currentDrone->allowed_to_goto[j] != 0)
            {

                sprintf(intAsString, "s%d", currentDrone->allowed_to_goto[j]);

                // if yes give it avalue if not (-1) inf
                if (strcmp(neighbors->keys[i], intAsString) == 0)
                {
                    // if the spot is free
                    if (strcmp(neighbors->Status[i], "f") == 0)
                    {
                        neighbors->spot_priority[i] = neighbors->distances[i] * C / denom;

                        break;
                    }
                    else // the spot is occupied
                    {

                        // no need to check if the deon is close to the sink or not
                        int isclose = 0;
                        // check if the spot is close to the sink
                        for (int k = 0; k < closeSinkSize; k++)
                        {
                            if (strcmp(neighbors->keys[i], closeSink[k]) == 0) // the spot is close to sink
                            {
                                isclose = 1;
                                break;
                            }
                        }
                        int occupiedByborder = 0;
                        for (int k = 0; k < borderNeigboorSize; k++)
                        {
                            if (strcmp(neighbors->keys[i], borderNeigboor[k]) == 0)
                            {
                                occupiedByborder = 1;
                                break;
                            }
                        }
                        if (isclose) // it is close to the sink should it should  be avoided
                        {
                            neighbors->spot_priority[i] = -1; // it is lik INF so it will not be consider when you find the minimum
                        }
                        else // spot occupied but far from the sink
                        {
                            neighbors->spot_priority[i] = randomFloat((neighbors->spot_num_drones[i] * C + eps), (neighbors->spot_num_drones[i] + 1) * C); // spot is away  from the sink
                            break;
                        }
                    }
                }
                else if (strcmp(neighbors->keys[i], intAsString) != 0) // if it it not allowed to go then set the priority as -1 (inf) so no go to it
                {
                    neighbors->spot_priority[i] = -1;
                }
            }
            else
            {
                neighbors->spot_priority[i] = -1;
            }
        }
    }
}

int countdronesAtPosition(Drones drones[], int numdrones, float x, float y)
{
    int count = 0;
    for (int i = 0; i < numdrones; i++)
    {
        if (float_compare(drones[i].x, x) && float_compare(drones[i].y, y))
        {

            count++;
        }
    }
    return count;
}

int countdronesAtPosition_with_specific_state(Drones drones[], int numdrones, float x, float y, int states)
{
    int count = 0;
    for (int i = 0; i < numdrones; i++)
    {
        if (float_compare(drones[i].x, x) && float_compare(drones[i].y, y) && (drones[i].state == Border || drones[i].state == Irremovable_border))
        {
            count++;
        }
    }
    return count;
}

void moveDrones(Drones *Drones, enum Direction dir)
{
    // Move the given Drones a fixed distance in one of the 6 directions
    Drones->x = round_to_decimals_float(Drones->x + DIR_VECTORS[dir][0], 2);
    Drones->y = round_to_decimals_float(Drones->y + DIR_VECTORS[dir][1], 2);
    Drones->drone_distance = sqrt((Drones->x * Drones->x) + (Drones->y * Drones->y));
}

void set_num_drones_at_neighbors(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones)
{
    int count_drons = 0;
    for (int j = 0; j < 7; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
    {

        count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
        if (count_drons > 0)
        {
            setStatus(neighbors, neighbors->keys[j], "o", count_drons);
        }
        else
        {
            setStatus(neighbors, neighbors->keys[j], "f", count_drons);
        }
    }
    count_drons = 0;
}

void check_drone_spot(Drones drones[], Drones *currentDrones, int numdrones)
{
    int previous_state = currentDrones->state;
    int count_drons = 0;

    count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[0][0], currentDrones->y + DIR_VECTORS[0][1]);
    if (count_drons > 1)
    {
        currentDrones->state = Free;
    }
    else
    {
        currentDrones->state = Alone;
    }
}

void check_drone_spot_further_expan(Drones drones[], Drones *currentDrones, int numdrones)
{
    int count_drons = 0;
    int previous_state = currentDrones->state;

    if (currentDrones->state == Free) // if the drone is free ( not border or irrmovable)
    {
        // check if the current spot contains more than one drone or not ( find if it is alon or no)
        count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[0][0], currentDrones->y + DIR_VECTORS[0][1]);
        if (count_drons > 1)
        {
            currentDrones->state = Free; // it is not alone but free
        }
        else
        {
            currentDrones->state = Alone; // it is alone
        }
    }
}

void find_border_update_drone_state(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones)
{
    int previous_state = currentDrones->state;
    int num_found_drones = 0;
    int count_drons = 0; // from 0 to 6 because s0 not needed since the dron is alone and we want to see around
    int edit_state = 0;
    int cond_done = 0;

    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
    {
        num_found_drones = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
        count_drons += num_found_drones;
    }
    // the drone at the sink is always irrmovable
    if (currentDrones->x == 0.0 && currentDrones->y == 0.0)
    {
        currentDrones->state = Irremovable;
    }
    else if (count_drons == 6)
    {
        currentDrones->state = Free; // free state
    }
    else if (count_drons < 6 && currentDrones->state == 0)
    {
        currentDrones->state = Border;
    }
}

void find_border_further_update_drone_state(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones)
{
    int previous_state = currentDrones->state;
    int num_found_drones = 0;
    int count_drons_border = 0; // from 0 to 6 because s0 not needed since the dron is alone and we want to see around
    int count_drons_alone = 0;
    int edit_state = 0;
    int cond_done = 0;
    if (currentDrones->state == Border || currentDrones->state == Irremovable_border)
    {
        {
            for (int j = 0; j < currentDrones->allowed_neighborsSize; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
            {
                int spot = currentDrones->allowed_to_goto[j];
                num_found_drones = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[spot][0], currentDrones->y + DIR_VECTORS[spot][1]);
                count_drons_border += num_found_drones;
            }
        }
    }
    if (currentDrones->state == Free || currentDrones->state == Alone)
    {
        {
            for (int j = 0; j < 7; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
            {
                num_found_drones = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
                count_drons_alone += num_found_drones;
            }
        }
    }

    if (currentDrones->x == 0.0 && currentDrones->y == 0.0)
    {
        currentDrones->state = Irremovable;
        for (int j = 0; j < 7; j++)
        {
            currentDrones->allowed_to_goto[j] = j;
        }
        currentDrones->allowed_neighborsSize = 7;
    }
    else if (count_drons_border == currentDrones->allowed_neighborsSize && (currentDrones->state == Border || currentDrones->state == Irremovable_border))
    {

        if (currentDrones->state == Border)
        {
            currentDrones->state = Free;
        }
        if (currentDrones->state == Irremovable_border)
        {
            currentDrones->state = Irremovable;
        }
    }
    else if ((currentDrones->state == Free || currentDrones->state == Alone) && count_drons_alone < 7)
    {
        currentDrones->state = Border;
        for (int j = 0; j < 7; j++)
        {
            currentDrones->allowed_to_goto[j] = j;
        }
        currentDrones->allowed_neighborsSize = 7;
    }
    else if ((currentDrones->state == Free || currentDrones->state == Alone) && count_drons_alone == 7)
    {
        currentDrones->state = Free;
    }
}

void perform_first_expansion(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, FILE *fp)
{
    char Priority[MAX_SIZE][MAX_SIZE];
    int PrioritySize;
    int dir = 0;
    int num_drone_alone = 0;
    int steps = 0;

    while (num_drone_alone < numdrones)
    {
        // each point now need to check around then move then the second oen do that
        num_drone_alone = 0;
        for (int i = 0; i < numdrones; i++)
        {
            check_drone_spot(drones, &drones[i], numdrones); // check if the drone is alone or nots
            if (drones[i].state != Alone)                    // drone is not alone
            {
                setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration

                set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
                setPriorities(&DroneNeighbors[i]);

                findPriority(&DroneNeighbors[i], Priority, &PrioritySize);

                sscanf(Priority[0], "s%d", &dir);
                moveDrones(&drones[i], dir);
                setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration
            }
            else // if the drone is alone do not move
            {
                num_drone_alone++;
            }
        }
    }
    saveDrones(drones, numdrones, fp);
}

void perform_further_expansion(Drones *drones, struct Neighbors *DroneNeighbors, int numdrones, FILE *fp)
{
    char Priority[MAX_SIZE][MAX_SIZE];
    int PrioritySize;
    int dir = 0;
    int num_drone_alone = 0;
    int steps = 0;
    //  another expansion
    // the drone after the balancing pahse will be on the border so many drones will be in same pspot
    while (num_drone_alone < numdrones)
    {
        // each point now need to check around then move then the second oen do that
        // the ones before should be moved so the next one can detect the new position
        num_drone_alone = 0;
        for (int i = 0; i < numdrones; i++)
        {
            check_drone_spot_further_expan(drones, &drones[i], numdrones);
            if (drones[i].state == Free)
            {
                setDist(&DroneNeighbors[i], drones[i].x, drones[i].y);
                set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);
                setPriorities_further_expan(drones, &drones[i], &DroneNeighbors[i], numdrones);
                findPriority(&DroneNeighbors[i], Priority, &PrioritySize);
                sscanf(Priority[0], "s%d", &dir);
                moveDrones(&drones[i], dir);
                setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration
            }
            else // if the drone is alone do not move and save that one more is alone
            {
                num_drone_alone++;
            }
        }
    }
    saveDrones(drones, numdrones, fp);
}

void form_border_and_update_states(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, Target targets[], int targets_size, FILE *fp)
{
    //  After the Drones spread,  now it is time to see if each drone is free or border
    for (int i = 0; i < numdrones; i++)
    {
        find_border_update_drone_state(drones, &DroneNeighbors[i], &drones[i], numdrones);
        set_state_target_check(drones, &drones[i], targets, targets_size, numdrones);
    }
    saveDrones(drones, numdrones, fp);
}

void form_further_border_and_update_states(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, Target targets[], int targets_size, FILE *fp)
{
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state != Irremovable)
        {
            find_border_further_update_drone_state(drones, &DroneNeighbors[i], &drones[i], numdrones);
        }
        set_state_target_check(drones, &drones[i], targets, targets_size, numdrones);
    }
    saveDrones(drones, numdrones, fp);
}