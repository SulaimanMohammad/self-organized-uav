#include "expansion.h"

// #define N 10 // Grid size

// #define N 10 // Grid size

// Define the unit vectors for each direction
// which is the 6 negihboors with respect of the drone position

double DIR_VECTORS[7][2] = {
    {0, 0},                             // s0 // dont move stay
    {(sqrt(3) * a), 0},                 // s1
    {(sqrt(3) / 2) * a, (3 / 2) * a},   // s2
    {-(sqrt(3) / 2) * a, (3 / 2) * a},  // s3
    {-sqrt(3) * a, 0},                  // s4
    {-(sqrt(3) / 2) * a, -(3 / 2) * a}, // s5
    {(sqrt(3) / 2) * a, -(3 / 2) * a}   // s6

};
// Initialize the dictionary
void initDictionary(struct Dictionary *dict)
{
    dict->size = 0;
}

// Add a new key-value pair to the dictionary
void addEntry(struct Dictionary *dict, char key[], float value)
{
    if (dict->size >= MAX_SIZE)
    {
        printf("Error: Dictionary is full\n");
        return;
    }
    strcpy(dict->keys[dict->size], key);
    dict->distances[dict->size] = value;
    dict->size++;
}

// this create the 6 spots with coordiantes regaring the sink not with the respect of the drone position
void creatSpots(struct Dictionary *dict, const float Dx, const float Dy)
{
    // calculated
    float DxDy2 = (Dx * Dx) + (Dy * Dy);
    float DxDy3a2 = DxDy2 + 3 * a * a;
    float sqDx = sqrt(3) * Dx;
    float aDx = (2 * sqrt(3)) * Dx;

    addEntry(dict, "s0", sqrt(DxDy2));
    addEntry(dict, "s1", sqrt(DxDy3a2 + a * aDx));
    addEntry(dict, "s2", sqrt(DxDy3a2 + a * (sqDx + (3 * Dy))));
    addEntry(dict, "s3", sqrt(DxDy3a2 + a * (3 * Dy - aDx)));
    addEntry(dict, "s4", sqrt(DxDy3a2 - aDx + 3 * a * a));
    addEntry(dict, "s5", sqrt(DxDy3a2 - a * (aDx + 3 * Dy)));
    addEntry(dict, "s6", sqrt(DxDy3a2 - a * (3 * Dy - sqDx)));
}

void setDist(struct Dictionary *dict, const float Dx, const float Dy)
{
    float DxDy2 = (Dx * Dx) + (Dy * Dy);
    float DxDy3a2 = DxDy2 + 3 * a * a;
    float sqDx = sqrt(3) * Dx;
    float aDx = (2 * sqrt(3)) * Dx;

    for (int i = 0; i < dict->size; i++)
    {
        if (strcmp(dict->keys[i], "s0") == 0)
            dict->distances[i] = sqrt(DxDy2);
        if (strcmp(dict->keys[i], "s1") == 0)
            dict->distances[i] = sqrt(DxDy3a2 + a * aDx);
        if (strcmp(dict->keys[i], "s2") == 0)
            dict->distances[i] = sqrt(DxDy3a2 + a * (sqDx + (3 * Dy)));
        if (strcmp(dict->keys[i], "s3") == 0)
            dict->distances[i] = sqrt(DxDy3a2 + a * (3 * Dy - aDx));
        if (strcmp(dict->keys[i], "s4") == 0)
            dict->distances[i] = sqrt(DxDy3a2 - aDx + 3 * a * a);
        if (strcmp(dict->keys[i], "s5") == 0)
            dict->distances[i] = sqrt(DxDy3a2 - a * (aDx + 3 * Dy));
        if (strcmp(dict->keys[i], "s6") == 0)
            dict->distances[i] = sqrt(DxDy3a2 - a * (3 * Dy - sqDx));
    }
}

void setStatus(struct Dictionary *dict, char key[], char state[], int W)
{
    // Search for the key in the dictionary
    for (int i = 0; i < dict->size; i++)
    {
        if (strcmp(dict->keys[i], key) == 0)
        {
            // Key already exists, update the value
            strcpy(dict->Status[i], state);
            dict->w[i] = W;
            return;
        }
    }

    // Key doesn't exist, add a new key-value pair
    // addEntry(dict, key, value);
}

// Get the value associated with a key in the dictionary
float getDist(struct Dictionary *dict, char key[])
{
    for (int i = 0; i < dict->size; i++)
    {
        if (strcmp(dict->keys[i], key) == 0)
        {
            return dict->distances[i];
        }
    }
    printf("Error: Key not found\n");
    return -1;
}

float getPriority(struct Dictionary *dict, char key[])
{
    for (int i = 0; i < dict->size; i++)
    {
        if (strcmp(dict->keys[i], key) == 0)
        {
            return dict->v[i];
        }
    }
    printf("Error: Key not found\n");
    return -1;
}

char *getStatus(struct Dictionary *dict, char key[])
{
    for (int i = 0; i < dict->size; i++)
    {
        if (strcmp(dict->keys[i], key) == 0)
        {
            return dict->Status[i];
        }
    }
    printf("Error: Key not found\n");
    return "\0"; // Return a null character to indicate an error;
}

// return the key of object in the minimum distance
void findMinDistances(struct Dictionary *dict, char result[MAX_SIZE][MAX_SIZE], int *resultSize)
{
    float minDistance = -1.0;
    int count = 0;

    // Iterate over all entries in the dictionary
    for (int i = 0; i < dict->size; i++)
    {
        // If the distance is positive and less than the current minimum, update the minimum and reset the result array
        if (dict->distances[i] > 0 && (minDistance < 0 || dict->distances[i] < minDistance))
        {
            minDistance = dict->distances[i];
            count = 0;
            strcpy(result[count], dict->keys[i]);
            count++;
        }
        // If the distance is equal to the current minimum, add the key to the result array
        else if (dict->distances[i] == minDistance)
        {
            strcpy(result[count], dict->keys[i]);
            count++;
        }
    }

    *resultSize = count;
}

void findPriority(struct Dictionary *dict, char result[MAX_SIZE][MAX_SIZE], int *resultSize)
{
    float minDistance = -1.0;
    int count = 0;

    // Iterate over all entries in the dictionary
    for (int i = 0; i < dict->size; i++)
    {
        // If the distance is positive and less than the current minimum, update the minimum and reset the result array
        if (dict->v[i] > 0 && (minDistance < 0 || dict->v[i] < minDistance))
        {
            minDistance = dict->v[i];
            count = 0;
            strcpy(result[count], dict->keys[i]);
            count++;
        }
        // If the distance is equal to the current minimum, add the key to the result array
        else if (dict->v[i] == minDistance)
        {
            strcpy(result[count], dict->keys[i]);
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

void setPriorities(struct Dictionary *dict)
{
    // find the spots closer to the sink to decide how to set the priorities
    char closeSink[MAX_SIZE][MAX_SIZE];
    int closeSinkSize;

    findMinDistances(dict, closeSink, &closeSinkSize);

    // drone now will set the value of vi that is assiged to each niegboor
    float denom = (4.0 * getDist(dict, "s0")); //
    for (int i = 0; i < dict->size; i++)
    {
        // if the spot is free
        if (strcmp(dict->Status[i], "f") == 0)
        {
            dict->v[i] = dict->distances[i] * C / denom;
        }
        else // the spot is occupied
        {
            int isclose = 0;
            // check if the spot is close to the sink
            for (int j = 0; j < closeSinkSize; j++)
            {
                if (strcmp(dict->keys[i], closeSink[j]) == 0) // the spot is close to sink
                {
                    isclose = 1;
                    break;
                }
            }
            if (isclose) // it is close to the sink should it should  be avoided
            {
                dict->v[i] = -1; // it is lik INF so it will not be consider when you find the minimum
            }
            else // spot occupied but far from the sink
            {
                dict->v[i] = randomFloat((dict->w[i] * C + eps), (dict->w[i] + 1) * C); // spot is away  from the sink
            }
        }
    }
}

void initializePoints(Point points[], int numPoints)
{
    // Initialize all points at the center of the grid
    for (int i = 0; i < numPoints; i++)
    {
        points[i].x = 0.;  // randomFloat(0, 1);
        points[i].y = 0.0; // randomFloat(0, 1);
    }
}

void printPoints(Point points[], int numPoints)
{
    for (int i = 0; i < numPoints; i++)
    {
        printf("(%f, %f), ", points[i].x, points[i].y);
    }
    printf("\n");
}

void savePoints(Point points[], int numPoints, FILE *fp)
{
    for (int i = 0; i < numPoints; i++)
    {
        fprintf(fp, "(");
        fprintf(fp, "%.6f", points[i].x);
        fprintf(fp, ", ");
        fprintf(fp, "%.6f", points[i].y);
        fprintf(fp, "),");
    }
    fprintf(fp, "\n");
}

int countPointsAtPosition(Point points[], int numPoints, float x, float y)
{
    int count = 0;
    for (int i = 0; i < numPoints; i++)
    {
        if (points[i].x == x && points[i].y == y)
        {
            count++;
        }
    }
    return count;
}

void movePoint(Point *point, enum Direction dir)
{
    // Move the given point a fixed distance in one of the 6 directions
    point->x += DIR_VECTORS[dir][0];
    point->y += DIR_VECTORS[dir][1];
}

void set_num_drones_at_neighbors(Point points[], struct Dictionary *dict, Point *currentPoint, int numPoints)
{
    int count_drons = 0;
    for (int j = 0; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the point
    {

        count_drons = countPointsAtPosition(points, numPoints, currentPoint->x + DIR_VECTORS[j][0], currentPoint->y + DIR_VECTORS[j][1]);
        printf(" point at (%f, %f ) has %d  drons at x %f, y %f\n", currentPoint->x, currentPoint->y, count_drons, currentPoint->x + DIR_VECTORS[j][0], currentPoint->y + DIR_VECTORS[j][1]);
        if (count_drons != 0)
        {
            setStatus(dict, dict->keys[j], "o", count_drons); // dictionaries[i].w[j]
        }
        else
        {
            setStatus(dict, dict->keys[j], "f", count_drons);
        }
    }
    count_drons = 0;
}
