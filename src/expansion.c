#include "expansion.h"
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
        // printf("spot %s, dis=%f , stat %s , num %d , prior %f \n", neighbors->keys[i], neighbors->distances[i], neighbors->Status[i], neighbors->w[i], neighbors->v[i]);
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
        drones[i].state = 0; // free
        drones[i].num_steps = 0;
        drones[i].direction_taken = (int *)malloc(0 * sizeof(int)); // allocate memoy
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
        // printf("neighbors S%d has %d  drons at x %f, y %f\n", j, count_drons, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
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
    int count_drons = 0;

    count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[0][0], currentDrones->y + DIR_VECTORS[0][1]);
    // printf("current drone S%d has %d  drons at x %f, y %f\n\n", 0, count_drons, currentDrones->x + DIR_VECTORS[0][0], currentDrones->y + DIR_VECTORS[0][1]);
    if (count_drons > 1)
    {
        currentDrones->state = 1; // it is not alone
    }
    else
    {
        currentDrones->state = 0; // it is alone
    }
}

void append_new_step(Drones *currentDrones, int dir)
{
    if (currentDrones->direction_taken == NULL)
    {
        // allocation error
        printf("Error: Memory allocation failed.\n");
    }
    else
    {
        // Memory allocation was successful

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
}

void find_border_update_drone_state(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones)
{
    int count_drons = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {
        count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
    }
    // the drone at the sink is always irrmovable
    // that i important for the connectivity
    if (currentDrones->x == 0.0 && currentDrones->y == 0.0)
    {
        currentDrones->state = 3;
    }
    if (count_drons == 6)
    {
        currentDrones->state = 0; // free state
    }
    else
    { // check if the drone is border=2
        // check if there are less than 3 spot occupied in the direction of the expansion
        int count_drons = -1;
        int direction = countElementOccurrences(currentDrones);
        switch (direction)
        {
        case 1: // most direction to s1 , then checl s1,s6, s2 to see the border
            count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[1][0], currentDrones->y + DIR_VECTORS[1][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[2][0], currentDrones->y + DIR_VECTORS[2][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[6][0], currentDrones->y + DIR_VECTORS[6][1]);
            if (count_drons < 3)
                currentDrones->state = 2;
            break;
        case 2:
            count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[1][0], currentDrones->y + DIR_VECTORS[1][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[2][0], currentDrones->y + DIR_VECTORS[2][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[3][0], currentDrones->y + DIR_VECTORS[3][1]);
            if (count_drons < 3)
                currentDrones->state = 2;
            break;
        case 3:
            count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[3][0], currentDrones->y + DIR_VECTORS[3][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[2][0], currentDrones->y + DIR_VECTORS[2][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[4][0], currentDrones->y + DIR_VECTORS[4][1]);
            if (count_drons < 3)
                currentDrones->state = 2;
            break;
        case 4:
            count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[3][0], currentDrones->y + DIR_VECTORS[3][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[5][0], currentDrones->y + DIR_VECTORS[5][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[4][0], currentDrones->y + DIR_VECTORS[4][1]);
            if (count_drons < 3)
                currentDrones->state = 2;
            break;
        case 5:
            count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[6][0], currentDrones->y + DIR_VECTORS[6][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[5][0], currentDrones->y + DIR_VECTORS[5][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[4][0], currentDrones->y + DIR_VECTORS[4][1]);
            if (count_drons < 3)
                currentDrones->state = 2;
            break;
        case 6:
            count_drons = countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[6][0], currentDrones->y + DIR_VECTORS[6][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[5][0], currentDrones->y + DIR_VECTORS[5][1]);
            count_drons += countdronesAtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[1][0], currentDrones->y + DIR_VECTORS[1][1]);
            if (count_drons < 3)
                currentDrones->state = 2;
            break;
        default:
            printf(" not border");
            break;
        }
    }
}

// check how many times each direction is repeated and give more wight to the last one
// return the most repeated after which will give idea about the direction of the expansion
int countElementOccurrences(const Drones *currentDrones)
{
    // Find the maximum element in the direction_taken array
    int max_element = 0;
    for (int i = 0; i < currentDrones->num_steps; i++)
    {
        if (currentDrones->direction_taken[i] > max_element)
        {
            max_element = currentDrones->direction_taken[i];
        }
    }

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
        // give the last step direction more wight and effect
        if (i == currentDrones->num_steps - 1)
        {
            int current_element = currentDrones->direction_taken[i];
            frequency_array[current_element] += 2;
        }
        else
        {
            int current_element = currentDrones->direction_taken[i];
            frequency_array[current_element]++;
        }
    }

    // Find the maximum occurrence count
    int max_occurrence = 0;
    int res = 0;
    for (int i = 0; i <= max_element; i++)
    {
        if (frequency_array[i] > max_occurrence)
        {
            max_occurrence = frequency_array[i];
            res = i;
        }
    }

    free(frequency_array);
    return res;
}

// scan hexagon by goning in smaller hex
int target_in_area(Drones *currentDrones, Target *targets, int targets_num, int distance, int length)
{
    int found = 0;
    float radius = a;
    double epsilon = 0.0000001;
    float angle = 0.0;
    float angleIncrement = 60.0; // Angle between vertices
    double nextY, nextX, deltx, deltY, total_distance, directionX, directionY, incrementX, incrementY = 0;
    // Start at the center
    double currentY = currentDrones->y;
    double currentX = currentDrones->x;
    int i = 0;
    while (radius - distance > 0)
    {
        // Move to the first vertex ( 12 oclock)
        currentY = round(currentDrones->y + (radius - distance) * cos(angle * 3.14 / 180.0) * 100) / 100;
        currentX = round(currentDrones->x + (radius - distance) * sin(angle * 3.14 / 180.0) * 100) / 100;

        for (int j = 0; j < 6; j++)
        {
            // Calculate the coordinates of the next neighboring vertex
            angle += angleIncrement;
            nextY = round(currentDrones->y + (radius - distance) * cos(angle * 3.14 / 180.0) * 100) / 100;
            nextX = round(currentDrones->x + (radius - distance) * sin(angle * 3.14 / 180.0) * 100) / 100;

            deltx = round((nextX - currentX) * 100) / 100;
            deltY = round((nextY - currentY) * 100) / 100;
            total_distance = round(sqrt(deltx * deltx + deltY * deltY) * 100) / 100;
            directionX = round((deltx / total_distance) * 100) / 100;
            directionY = round((deltY / total_distance) * 100) / 100;
            incrementX = round(directionX * length * 100) / 100;
            incrementY = round(directionY * length * 100) / 100;

            // printf("j= %d go (%f, %f) to (%f,%f) with dirx= %f, diry=%f icx= %f, incy= %f\n\n\n",j,currentX, currentY,nextX,nextY,directionX,directionY, incrementX, incrementY );

            if (directionX > 0 && directionY < 0)
            {
                while (currentX <= nextX && currentY >= nextY)
                {
                    currentY += incrementY;
                    currentX += incrementX;
                    // printf("(%d, %f, %f, %d),", i, currentX, currentY, 0);
                    for (int k = 0; k < targets_num; k++)
                    {
                        // printf("tx= %f , x %f, ty=%f ,y%f \n", targets[k].x, currentX, targets[k].y, currentY);

                        if (currentX == targets[k].x && currentY == targets[k].y)
                        {
                            found = 1;
                            break;
                        }
                    }
                }
            }
            if (directionX < epsilon && directionY < 0)
            {
                while (currentX == nextX && currentY >= nextY)
                {
                    currentY += incrementY;
                    currentX += incrementX;

                    for (int k = 0; k < targets_num; k++)
                    {
                        // printf("tx %f, ty%f \n", targets[k].x, targets[k].y);

                        if (currentX == targets[k].x && currentY == targets[k].y)
                        {
                            found = 1;
                            break;
                        }
                    }
                }
            }
            if (directionX < 0 && directionY < 0)
            {
                while (currentX >= nextX && currentY >= nextY)
                {
                    currentY += incrementY;
                    currentX += incrementX;

                    for (int k = 0; k < targets_num; k++)
                    {
                        // printf("tx %f, ty%f \n", targets[k].x, targets[k].y);

                        if (currentX == targets[k].x && currentY == targets[k].y)
                        {
                            found = 1;
                            break;
                        }
                    }
                }
            }
            if (directionX < 0 && directionY > 0)
            {
                while (currentX >= nextX && currentY <= nextY)
                {
                    currentY += incrementY;
                    currentX += incrementX;

                    for (int k = 0; k < targets_num; k++)
                    {
                        // printf("tx %f, ty%f \n", targets[k].x, targets[k].y);

                        if (currentX == targets[k].x && currentY == targets[k].y)
                        {
                            found = 1;
                            break;
                        }
                    }
                }
            }
            if (directionX < epsilon && directionY > 0)
            {
                while (currentX == nextX && currentY <= nextY)
                {
                    currentY += incrementY;
                    currentX += incrementX;

                    for (int k = 0; k < targets_num; k++)
                    {
                        // printf("tx %f, ty%f \n", targets[k].x, targets[k].y);

                        if (currentX == targets[k].x && currentY == targets[k].y)
                        {
                            found = 1;
                            break;
                        }
                    }
                }
            }
            if (directionX > 0 && directionY > 0)
            {
                while (currentX <= nextX && currentY <= nextY)
                {
                    currentY += incrementY;
                    currentX += incrementX;

                    for (int k = 0; k < targets_num; k++)
                    {
                        // printf("tx %f, ty%f \n", targets[k].x, targets[k].y);

                        if (currentX == targets[k].x && currentY == targets[k].y)
                        {
                            found = 1;
                            break;
                        }
                    }
                }
            }

            currentY = nextY;
            currentX = nextX;
        }
        radius = radius - distance;
        i++;
    }
    return found;

    // // Return to the center
    // currentX = centerX;
    // currentY = centerY;
    // printf("(%f, %f)\n", currentX, currentY);
}
