#include "expansion.h"
#include "spanning.h" // needed for set_state_target_check in the expansion
#include <stdbool.h>

// Define the unit vectors for each direction
// which is the 6 negihboors with respect of the drone position
#define sqrt3 1.7
float DIR_VECTORS[7][2] = {
    {0, 0},                                 // s0 // dont move stay
    {(sqrt3 * a), 0},                       // s1
    {(sqrt3 / 2.0) * a, (3.0 / 2.0) * a},   // s2
    {-(sqrt3 / 2.0) * a, (3.0 / 2.0) * a},  // s3
    {-sqrt3 * a, 0},                        // s4
    {-(sqrt3 / 2.0) * a, -(3.0 / 2.0) * a}, // s5
    {(sqrt3 / 2.0) * a, -(3.0 / 2.0) * a}   // s6

};
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

// this create the 6 spots with coordiantes regaring the sink not with the respect of the drone position
void creatSpots(struct Neighbors *neighbors, float Dx, float Dy)
{
    // calculated
    float DxDy2 = round(((Dx * Dx) + (Dy * Dy)) * 100) / 100;
    float DxDy3a2 = round((DxDy2 + (3 * a * a)) * 100) / 100;
    float sqDx = round((sqrt3 * Dx) * 100) / 100;
    float aDx = round(((2 * sqrt3) * Dx) * 100) / 100;

    addEntry(neighbors, "s0", round(sqrt(DxDy2) * 100) / 100);
    addEntry(neighbors, "s1", round(sqrt(DxDy3a2 + a * aDx) * 100) / 100);
    addEntry(neighbors, "s2", round(sqrt(DxDy3a2 + a * (sqDx + (3 * Dy))) * 100) / 100);
    addEntry(neighbors, "s3", round(sqrt(DxDy3a2 + a * (3 * Dy - sqDx)) * 100) / 100);
    addEntry(neighbors, "s4", round(sqrt(DxDy3a2 - aDx * a) * 100) / 100);
    addEntry(neighbors, "s5", round(sqrt(DxDy3a2 - a * (sqDx + (3 * Dy))) * 100) / 100);
    addEntry(neighbors, "s6", round(sqrt(DxDy3a2 - a * (3 * Dy - sqDx)) * 100) / 100);
}

void setDist(struct Neighbors *neighbors, float Dx, float Dy)
{
    float DxDy2 = round(((Dx * Dx) + (Dy * Dy)) * 100) / 100;
    float DxDy3a2 = round((DxDy2 + (3 * a * a)) * 100) / 100;
    float sqDx = round((sqrt3 * Dx) * 100) / 100;
    float aDx = round(((2 * sqrt3) * Dx) * 100) / 100;

    for (int i = 0; i < neighbors->size; i++)
    {
        if (strcmp(neighbors->keys[i], "s0") == 0)
            neighbors->distances[i] = round(sqrt(DxDy2) * 100) / 100;
        if (strcmp(neighbors->keys[i], "s1") == 0)
            neighbors->distances[i] = round(sqrt(DxDy3a2 + a * aDx) * 100) / 100;
        if (strcmp(neighbors->keys[i], "s2") == 0)
            neighbors->distances[i] = round(sqrt(DxDy3a2 + a * (sqDx + (3 * Dy))) * 100) / 100;
        if (strcmp(neighbors->keys[i], "s3") == 0)
            neighbors->distances[i] = round(sqrt(DxDy3a2 + a * (3 * Dy - sqDx)) * 100) / 100;
        if (strcmp(neighbors->keys[i], "s4") == 0)
            neighbors->distances[i] = round(sqrt(DxDy3a2 - aDx * a) * 100) / 100;
        if (strcmp(neighbors->keys[i], "s5") == 0)
            neighbors->distances[i] = round(sqrt(DxDy3a2 - a * (sqDx + (3 * Dy))) * 100) / 100;
        if (strcmp(neighbors->keys[i], "s6") == 0)
            neighbors->distances[i] = round(sqrt(DxDy3a2 - a * (3 * Dy - sqDx)) * 100) / 100;
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
            neighbors->w[i] = W;
            return;
        }
    }

    // Key doesn't exist, add a new key-value pair
    // addEntry(neighbors, key, value);
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
            return neighbors->v[i];
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
        if (neighbors->v[i] >= 0 && (minDistance < 0 || neighbors->v[i] < minDistance))
        {
            minDistance = neighbors->v[i];
            count = 0;
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
        // If the distance is equal to the current minimum, add the key to the result array
        else if (neighbors->v[i] == minDistance)
        {
            strcpy(result[count], neighbors->keys[i]);
            count++;
        }
    }

    *resultSize = count;
}

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
            neighbors->v[i] = neighbors->distances[i] * C / denom;
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
                neighbors->v[i] = -1; // it is lik INF so it will not be consider when you find the minimum
            }
            else // spot occupied but far from the sink
            {
                neighbors->v[i] = randomFloat((neighbors->w[i] * C + eps), (neighbors->w[i] + 1) * C); // spot is away  from the sink
            }
        }
    }
}

// this function find what was the dominated path to detect the expansion path
// based on that we can define what neighbors we can go
void allowed_neigbors_for_further_expan(Drones drones[], Drones *currentDrone, char allowed_neighbors[MAX_SIZE][MAX_SIZE], int *allowed_neighborsSize, int numdrones)
{
    int count = 0;
    // if the drone in same spot as a border drone then it should not take the direction behind the border
    for (int i = 0; i < numdrones; i++)
    {

        if (drones[i].x == currentDrone->x && drones[i].y == currentDrone->y && (drones[i].state == 2 || drones[i].state == 3 || drones[i].state == 4))
        {
            // printf(" drone %d is in the spot of border %d \n ", currentDrone->id, drones[i].id);
            //  find what direction is missing in the array of border_neighbors of the border drone
            int fullArr[] = {1, 2, 3, 4, 5, 6};
            int fullSize = 6;

            bool found[fullSize];
            for (int j = 0; j < fullSize; j++)
            {
                found[j] = false;
            }

            // Mark numbers found in the given array
            for (int j = 0; j < drones[i].num_neighbors; j++)
            {
                found[drones[i].border_neighbors[j] - 1] = true;
            }

            // Find missing numbers
            //*numMissing = 0;
            for (int j = 0; j < fullSize; j++)
            {
                if (!found[j])
                {
                    char neighbors[3];
                    sprintf(neighbors, "s%d", j + 1);
                    strcpy(allowed_neighbors[count], neighbors);
                    count++;
                }
            }
            *allowed_neighborsSize = count;
        }
    }
    if (*allowed_neighborsSize > 0) // that means that the drone was in the spot of the border and that will happen only one time after the spanning
        return;
    else
    {
        *allowed_neighborsSize = 3;
        int count_drons = -1;
        int dom_direction = countElementOccurrences(currentDrone);
        switch (dom_direction)
        {
        case 1: // most direction to s1 , then checl s1,s6, s2 allowed
            strcpy(allowed_neighbors[0], "s1");
            strcpy(allowed_neighbors[1], "s2");
            strcpy(allowed_neighbors[2], "s6");
            break;
        case 2: // most direction to s2 , then checl s1,s2, s3 allowed
            strcpy(allowed_neighbors[0], "s1");
            strcpy(allowed_neighbors[1], "s2");
            strcpy(allowed_neighbors[2], "s3");
            break;
        case 3: // most direction to s3 , then checl s2,s3, s4 allowed
            strcpy(allowed_neighbors[0], "s4");
            strcpy(allowed_neighbors[1], "s2");
            strcpy(allowed_neighbors[2], "s3");
            break;
        case 4: // most direction to s4 , then checl s5,s3, s4 allowed
            strcpy(allowed_neighbors[0], "s5");
            strcpy(allowed_neighbors[1], "s4");
            strcpy(allowed_neighbors[2], "s3");
            break;
        case 5: // most direction to s5 , then checl s4,s5, s6 allowed
            strcpy(allowed_neighbors[0], "s4");
            strcpy(allowed_neighbors[1], "s5");
            strcpy(allowed_neighbors[2], "s6");
            break;
        case 6: // most direction to s6 , then checl s6,s5, s1 allowed
            strcpy(allowed_neighbors[0], "s6");
            strcpy(allowed_neighbors[1], "s5");
            strcpy(allowed_neighbors[2], "s1");
            break;
        default: // direction most of time was s0
            strcpy(allowed_neighbors[0], "s2");
            strcpy(allowed_neighbors[1], "s5");
            strcpy(allowed_neighbors[2], "s1");
            break;
        }
    }
}

// the difference between this and the setPriorities , here the riority will be like inf(-1) for the neigboor that opposit of the expansion area
// NOTE: this function also can be used to prevent a drone to go somewhere specific by chosing the allowed_neighbors manually

void findBorderDroneAround(Drones drones[], Drones *currentDrone, char result[MAX_SIZE][MAX_SIZE], int *resultSize, int numdrones)
{

    int count = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (drones[i].x == currentDrone->x + DIR_VECTORS[j][0] && drones[i].y == currentDrone->y + DIR_VECTORS[j][1] && (drones[i].state == 2 || drones[i].state == 3 || drones[i].state == 4))
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

void setPriorities_further_expan(Drones drones[], Drones *currentDrone, struct Neighbors *neighbors, char allowed_neighbors[MAX_SIZE][MAX_SIZE], int allowed_neighborsSize, int numdrones)
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
    for (int i = 0; i < neighbors->size; i++) // the three allowed spots
    {
        count = 0;
        // for each neighbors check if it is part from the allowed to go
        for (int j = 0; j < allowed_neighborsSize; j++)
        {
            // if yes give it avalue if not (-1) inf
            if (strcmp(neighbors->keys[i], allowed_neighbors[j]) == 0)
            {
                // if the spot is free
                if (strcmp(neighbors->Status[i], "f") == 0)
                {
                    neighbors->v[i] = neighbors->distances[i] * C / denom;
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
                    if (isclose || occupiedByborder) // it is close to the sink should it should  be avoided
                    {
                        neighbors->v[i] = -1; // it is lik INF so it will not be consider when you find the minimum
                    }
                    else // spot occupied but far from the sink
                    {
                        neighbors->v[i] = randomFloat((neighbors->w[i] * C + eps), (neighbors->w[i] + 1) * C); // spot is away  from the sink
                        break;
                    }
                }
            }
            else if (strcmp(neighbors->keys[i], allowed_neighbors[j]) != 0) // if it it not allowed to go then set the priority as -1 (inf) so no go to it
            {
                count++;
            }
        }
        if (count == allowed_neighborsSize) // non of the niegbor are in the allowed direction, and done this way to prevefnt them form changing the value
        {
            neighbors->v[i] = -1;
        }
    }
}

void initializeDrones(Drones drones[], int numdrones)
{
    // Initialize all drones at the center of the grid
    for (int i = 0; i < numdrones; i++)
    {
        drones[i].id = i;
        drones[i].x = 0.0;   // randomFloat(0, 1);
        drones[i].y = 0.0;   // randomFloat(0, 1);
        drones[i].state = 1; // free
        drones[i].num_steps = 0;
        drones[i].direction_taken = (int *)malloc(0 * sizeof(int)); // allocate memoy
        drones[i].previous_state = 0;
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
        fprintf(fp, "%.6f", drones[i].x);
        fprintf(fp, ", ");
        fprintf(fp, "%.6f", drones[i].y);
        fprintf(fp, ", ");
        fprintf(fp, "%d", drones[i].state);
        fprintf(fp, "),");
    }
    fprintf(fp, "\n");
}

int countdronesAtPosition(Drones drones[], int numdrones, float x, float y)
{
    int count = 0;
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].x == x && drones[i].y == y)
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
        if (drones[i].x == x && drones[i].y == y && (drones[i].state == 2 || drones[i].state == 4))
        {
            count++;
        }
    }
    return count;
}

void moveDrones(Drones *Drones, enum Direction dir)
{
    // Move the given Drones a fixed distance in one of the 6 directions
    Drones->x = round((Drones->x + DIR_VECTORS[dir][0]) * 100) / 100;
    Drones->y = round((Drones->y + DIR_VECTORS[dir][1]) * 100) / 100;
}

void set_num_drones_at_neighbors(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones)
{
    int count_drons = 0;
    for (int j = 0; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {

        count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
        if (count_drons > 0)
        {
            setStatus(neighbors, neighbors->keys[j], "o", count_drons); // neighborsionaries[i].w[j]
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
        currentDrones->state = 1; // it is not alone
    }
    else
    {
        currentDrones->state = 0; // it is alone
    }

    // if (currentDrones->state != previous_state)
    //     currentDrones->previous_state = previous_state;
}

void check_drone_spot_further_expan(Drones drones[], Drones *currentDrones, int numdrones)
{
    int count_drons = 0;
    int previous_state = currentDrones->state;

    if (currentDrones->state == 1) // if the drone is free ( not border or irrmovable)
    {
        count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[0][0], currentDrones->y + DIR_VECTORS[0][1]);
        if (count_drons > 1)
        {
            currentDrones->state = 1; // it is not alone but free
        }
        else
        {
            currentDrones->state = 0; // it is alone
        }
    }

    // if (currentDrones->state != previous_state)
    //     currentDrones->previous_state = previous_state;
}

void append_new_step(Drones *currentDrones, int dir)
{
    // Add a new element to the dynamic array
    currentDrones->num_steps = currentDrones->num_steps + 1; // Increase the number of steps
    int *temp = (int *)realloc(currentDrones->direction_taken, (currentDrones->num_steps) * sizeof(int));

    if (temp == NULL)
    {
        // Handle reallocation error
        printf("Error: Memory reallocation failed.\n");
    }
    else
    {
        // Reallocation was successful
        currentDrones->direction_taken = temp;
        currentDrones->direction_taken[currentDrones->num_steps - 1] = dir; // Add the new element
    }
}

void append_drones_neighbors_names(Drones *currentDrones, int neighbor)
{
    for (int i = 0; i < currentDrones->num_neighbors; i++)
    {
        if (currentDrones->border_neighbors[i] == neighbor)
        {
            // Neighbor already exists in the array, no need to append
            return;
        }
    }
    // Neighbor not found, proceed with appending
    currentDrones->num_neighbors++;
    int *temp = (int *)realloc(currentDrones->border_neighbors, currentDrones->num_neighbors * sizeof(int));
    if (temp == NULL)
    {
        printf("Error: Memory reallocation failed.\n");
    }
    else
    {
        currentDrones->border_neighbors = temp;
        currentDrones->border_neighbors[currentDrones->num_neighbors - 1] = neighbor;
    }
}
/*
after the first expansion the steps list will be deleted
the reason is that when you need to use the steps in the next expansion it should be caclaulted based on the movement in spaning
so no need to use the direction of the expansion of the first expansion phase, because it is doen
the expansion direction now will be calcuated based on the movement after spanning

*/
void reset_steps(Drones *currentDrones)
{
    if (currentDrones->direction_taken != NULL && currentDrones->num_steps > 0)
    {
        free(currentDrones->direction_taken);
        currentDrones->direction_taken = NULL;
        currentDrones->num_steps = 0;
    }
}

void reset_drones_neighbors_names(Drones *currentDrones)
{
    if (currentDrones->border_neighbors != NULL && currentDrones->num_neighbors > 0)
    {
        free(currentDrones->border_neighbors);
        currentDrones->border_neighbors = NULL;
        currentDrones->num_neighbors = 0;
    }
}

void find_border_update_drone_state(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones)
{
    int previous_state = currentDrones->state;
    int num_found_drones = 0;
    int count_drons = 0; // from 0 to 6 because s0 not needed since the dron is alone and we want to see around
    int edit_state = 0;
    int cond_done = 0;

    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {
        num_found_drones = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
        if (num_found_drones != 0) // if there is drone found at a neighbor save that neighbor
            append_drones_neighbors_names(currentDrones, j);
        count_drons += num_found_drones;
    }
    // the drone at the sink is always irrmovable
    // that i important for the connectivity
    // the center always irrmovable
    if (currentDrones->x == 0.0 && currentDrones->y == 0.0) // TODO that should be doen only if no target found
    {
        currentDrones->state = 3;
        reset_drones_neighbors_names(currentDrones); // this is needed only for the border
    }
    else if (count_drons == 6)
    {
        currentDrones->state = 1;                    // free state
        reset_drones_neighbors_names(currentDrones); // this is needed only for the border
    }
    else if (currentDrones->state == 2) // for further ecpansion ( because in first expansion no drone will be in border)
    {
        if (currentDrones->num_neighbors == 6)
        {
            currentDrones->state = 1;
            reset_drones_neighbors_names(currentDrones); // need to be free so all can be restored in the next phase
        }
    }
    else
    { // if the drone is not already a border // because in the futher expansion the deone was in border will not have full niegbor around but they should get free
      // int the further phase it should should keep anything whatever

        // check if the drone is border=2
        // check if there are less than 3 spot occupied in the direction of the expansion
        int count_drons = -1;
        int direction = countElementOccurrences(currentDrones); // check th direction of the expansion of the drone
        // printf("Drone %d direction=%d\n", currentDrones->id, direction);
        currentDrones->direction_dominated = direction;
        switch (direction)
        {
        case 1: // most direction to s1 , then check s1,s6, s2 to see the border
            int to_check1[] = {1, 2, 6};
            memcpy(currentDrones->to_check, to_check1, sizeof(3));
            conditions_to_find_border(drones, currentDrones, numdrones, to_check1, direction);
            break;
        case 2:
            int to_check2[] = {1, 2, 3};
            memcpy(currentDrones->to_check, to_check2, sizeof(3));

            conditions_to_find_border(drones, currentDrones, numdrones, to_check2, direction);
            break;
        case 3:
            int to_check3[] = {3, 2, 4};
            memcpy(currentDrones->to_check, to_check3, sizeof(3));
            conditions_to_find_border(drones, currentDrones, numdrones, to_check3, direction);
            break;
        case 4:
            int to_check4[] = {3, 4, 5};
            memcpy(currentDrones->to_check, to_check4, sizeof(3));
            conditions_to_find_border(drones, currentDrones, numdrones, to_check4, direction);
            break;
        case 5:
            int to_check5[] = {4, 5, 6};
            memcpy(currentDrones->to_check, to_check5, sizeof(3));
            conditions_to_find_border(drones, currentDrones, numdrones, to_check5, direction);
            break;
        case 6:
            int to_check6[] = {1, 5, 6};
            memcpy(currentDrones->to_check, to_check6, sizeof(3));
            conditions_to_find_border(drones, currentDrones, numdrones, to_check6, direction);
            break;
        }
    }
}

void update_irrmovable_border_state(Drones drones[], Drones *currentDrones, int numdrones) // dont check will be =3
{
    int num_found_drones = 0;
    for (int i = 0; i < 3; i++)
    {
        if (currentDrones->border_irrrmovable[i] == 0) // means that the spot was not occupied before
        {
            // check if it is fullfiled
            num_found_drones = countdronesAtPosition(drones, numdrones,
                                                     currentDrones->x + DIR_VECTORS[currentDrones->to_check[i]][0],
                                                     currentDrones->y + DIR_VECTORS[currentDrones->to_check[i]][1]);
            if (num_found_drones > 0)
                currentDrones->border_irrrmovable[i] = 1;
        }
    }
}

void conditions_to_find_border(Drones drones[], Drones *currentDrones, int numdrones, int to_check[], int dominate_dir)
{
    int previous_state = currentDrones->state;
    int num_found_drones = 0;
    int count_drons = 0;
    int s;

    /* no need to check for 3 because alrady not entered
    notie here we check for the ones as 4 and check for it is around to know if it is still border or no */
    // if the drone is not any more in the border but it was border-rrmovable then chabge it to be only irrmovable

    if (currentDrones->state == 4)
    {
        /* suppose  to_check6[] = {1, 5, 6};
                    border_irrrmovabl= {1,1,0}
                    check if spot s6 is fullfiled
        */
        int change_count = 0;
        for (int i = 0; i < 3; i++)
        {
            if (currentDrones->border_irrrmovable[i] == 0) // means that the spot was not occupied before
            {
                // check if it is fullfiled
                num_found_drones = countdronesAtPosition(drones, numdrones,
                                                         currentDrones->x + DIR_VECTORS[to_check[i]][0],
                                                         currentDrones->y + DIR_VECTORS[to_check[i]][1]);
                if (num_found_drones > 0)
                {
                    currentDrones->border_irrrmovable[i] = 1;
                    change_count++;
                    //  because it can be all become 1 at some point then it should not goes to 3
                }
            }
        }
        int count = 0;
        for (int i = 0; i < 3; i++) //
        {
            if (currentDrones->border_irrrmovable[i] == 1)
                count++;
        }

        if (count == 3 && change_count > 0) // NOTE: this change count can lead to wrong answer
        {
            currentDrones->state = 3;
        }
    }
    else // for the rest of drones
    {
        for (int i = 0; i < 3; i++)
        {
            s = to_check[i];
            num_found_drones = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[s][0], currentDrones->y + DIR_VECTORS[s][1]);
            count_drons += num_found_drones;
        }
        if (count_drons < 3)
        {
            currentDrones->state = 2;
        }
        else
        {
            currentDrones->state = 1;
            reset_drones_neighbors_names(currentDrones); // this is needed only for the border
        }
    }
}

// check how many times each direction is repeated and give more wight to the last one
// return the most repeated after which will give idea about the direction of the expansion
int countElementOccurrences(const Drones *currentDrones)
{
    // Find the maximum element in the direction_taken array
    int max_element = 7; // s0 to s6

    // Create an array to store the frequencies
    int *frequency_array = (int *)calloc(max_element + 1, sizeof(int));

    if (frequency_array == NULL)
    {
        printf("Error: Memory allocation failed.\n");
        return 0;
    }

    // Count the occurrences of each element
    for (int i = 0; i < currentDrones->num_steps; i++)
    {
        /*  give the last step direction more wight and effect
        That was used before but no need to do it because anyway if the expansion is done will the path of the expansion will be the max ocurance
        and since this function will be used also in the further e
        */

        int current_element = currentDrones->direction_taken[i];
        frequency_array[current_element]++;
    }

    int max = frequency_array[1];
    int max_count = 1;
    int max_index = 1;
    // Find the maximum frequency
    for (int i = 2; i < max_element; i++)
    {
        if (frequency_array[i] > max)
        {
            max = frequency_array[i];
            max_count = 1;
            max_index = i;
        }
        else if (frequency_array[i] == max)
        {
            max_count++;
        }
    }
    // If there is only one occurrence of the maximum frequency, return its index
    if (max_count == 1)
    {
        free(frequency_array);
        return max_index;
    }

    // If there are multiple occurrences of the maximum frequency, choose randomly
    int random_index = rand() % max_count;
    int count = 0;

    // Find the randomly chosen occurrence of the maximum frequency and return its index
    for (int i = 1; i < max_element; i++)
    {
        if (frequency_array[i] == max)
        {
            count++;
            if (count == random_index + 1)
            {
                free(frequency_array);
                return i;
            }
        }
    }
}

void perform_first_expansion(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, FILE *fp)
{
    char Priority[MAX_SIZE][MAX_SIZE];
    int PrioritySize;
    int dir = 0;
    int num_drone_alone = 0;
    int steps = 0;

    // state=1 not alone
    // state = 0 it is alone and free
    while (num_drone_alone < numdrones)
    {
        // each point now need to check around then move then the second oen do that
        // the ones before should be moved so the next one can detect the new position
        num_drone_alone = 0;
        for (int i = 0; i < numdrones; i++)
        {
            // Drones *dronez_conidered = &drones[i];
            check_drone_spot(drones, &drones[i], numdrones); // check if the drone is alone or nots
            if (drones[i].state != 0)                        // drone is not alone    // to move and there are many drone in the same place
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
            //      if the droen is free                               // check if the drone is alone or nots
            if (drones[i].state == 1) // drone is not alone    // to move and there are many drone in the same place
            {
                setDist(&DroneNeighbors[i], drones[i].x, drones[i].y); // update for the next iteration
                                                                       // // printf("is it ok");
                set_num_drones_at_neighbors(drones, &DroneNeighbors[i], &drones[i], numdrones);

                int allowed_neighborsSize = 0;
                char allowed_to_goto[MAX_SIZE][MAX_SIZE];                                                                   // not that the size of that is always 3 because the direction is defiened only by 3 spots
                allowed_neigbors_for_further_expan(drones, &drones[i], allowed_to_goto, &allowed_neighborsSize, numdrones); // the drone include also the path arr
                // DroneNeighbors relate to drone[i]
                setPriorities_further_expan(drones, &drones[i], &DroneNeighbors[i], allowed_to_goto, allowed_neighborsSize, numdrones);
                findPriority(&DroneNeighbors[i], Priority, &PrioritySize); // that should return only one number, since there is also random number generator
                                                                           // if the random numbering is not considered then there will be many possible solution
                                                                           // and that if the spots has the same number of dron in it and using the [f(w,c,eps), f(w,c)[
                                                                           // then same number will be choosed for pts has same drons
                sscanf(Priority[0], "s%d", &dir);
                moveDrones(&drones[i], dir);
                append_new_step(&drones[i], dir);
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

void form_border_and_update_states(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, Target targets[], int targets_size, FILE *fp)
{
    //  After the Drones spread,  now it is time to see if each drone is free or border
    for (int i = 0; i < numdrones; i++)
    {
        find_border_update_drone_state(drones, &DroneNeighbors[i], &drones[i], numdrones);
        set_state_target_check(drones, &drones[i], targets, targets_size, numdrones);
        // do not free it here because it will be used agin
        // free(drones[i].direction_taken); // no need for it any more
        // after the expansion is done ther will be no need for the record of the path beuse it is done
        if (drones[i].state == 1)
            reset_steps(&drones[i]); // reset the path but keep it for the border because it will be used in next building of the border
    }
    saveDrones(drones, numdrones, fp);
}

void form_further_border_and_update_states(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, Target targets[], int targets_size, FILE *fp)
{
    for (int i = 0; i < numdrones; i++)
    {

        // int the further phase it should should keep anything whatever
        // only drone that has free alone state will be candidater of border
        // if you removed the condition the drones from previous spaning will be n middel nothing around will be considered as border and that is wrong

        /*here no need to put all the old drone to 3 instde we make the drone with 4 state ( bodere-irr)
        to check if it is still border or no , and the coditions are add to find_border_update_drone_state -> conditions_to_find_border
        and at the same time the one irrmovale will not enter because are possible in the middel
        and should not be recognized s border 1
        */

        if (drones[i].state != 3) // you should use && not || because || if it state 3 then it will enter because or evaluate the first on if t is true it will continue
        {

            find_border_update_drone_state(drones, &DroneNeighbors[i], &drones[i], numdrones);
            //  after the expansion is done ther will be no need for the record of the path because it is done
            if (drones[i].state == 1) // only free the drone with state free
                reset_steps(&drones[i]);
        }

        // the drone that was border and irrmovable they should be checked, because the border changed
        // so a drone if it was not part of the new border it should become just irrmovable
        // this should be done before build_path_to_border or the the new border will be connected
        // and that because the building stops based on recursion that contains state border ot border irrmobavale
        // so if the old irrmovable border of the old border then the rocess will stop before arriving to the new border

        set_state_target_check(drones, &drones[i], targets, targets_size, numdrones);
    }
    saveDrones(drones, numdrones, fp);
}
