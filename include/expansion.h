#ifndef EXPANSION_H
#define EXPANSION_H
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

#define C 100
#define eps 20
#define a 20
#define MAX_SIZE 7

enum Direction
{
    s0,
    s1,
    s2,
    s3,
    s4,
    s5,
    s6
};
extern float DIR_VECTORS[7][2];

typedef struct Drones
{
    float x;
    float y;
    int state;
    // alone=1 free=0 border=2 irrmovable=3
    int num_steps;
    int *direction_taken; // array contains the direction was taken

} Drones;

struct Neighbors
{
    char keys[MAX_SIZE][MAX_SIZE];
    float distances[MAX_SIZE];
    float v[MAX_SIZE];
    char Status[MAX_SIZE][MAX_SIZE]; // occupied or free
    int size;
    int w[MAX_SIZE]; // you needed to be incremented when a new arriver
};

// Initialize the Neighbors
void initNeighbors(struct Neighbors *neighbors);

// Add a new key-value pair to the Neighbors
void addEntry(struct Neighbors *neighbors, char key[], float value);

// this create the 6 spots with coordiantes regaring the sink not with the respect of the drone position
void creatSpots(struct Neighbors *neighbors, float Dx, float Dy);
void setDist(struct Neighbors *neighbors, float Dx, float Dy); // for each new iteration

void setStatus(struct Neighbors *neighbors, char key[], char state[], int W);

// Get the value associated with a key in the Neighbors
float getDist(struct Neighbors *neighbors, char key[]);

float getPriority(struct Neighbors *neighbors, char key[]);

char *getStatus(struct Neighbors *neighbors, char key[]);

// return the key of object in the minimum distance
void findMinDistances(struct Neighbors *neighbors, char result[MAX_SIZE][MAX_SIZE], int *resultSize);

void findPriority(struct Neighbors *neighbors, char result[MAX_SIZE][MAX_SIZE], int *resultSize);

// generate random number [A,B[ , b is not included
float randomFloat(float A, float B);

int randomInt(int A, int B);

void setPriorities(struct Neighbors *neighbors);

void initializeDrones(Drones drones[], int numdrones);

void printDrones(Drones drones[], int numdrones);
void saveDrones(Drones drones[], int numdrones, FILE *fp);

int countdronesAtPosition(Drones drones[], int numdrones, float x, float y);

void moveDrones(Drones *Drones, enum Direction dir);

void set_num_drones_at_neighbors(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones);
void check_drone_spot(Drones drones[], Drones *currentDrones, int numdrones);
void append_new_step(Drones *currentDrones, int dir);
void update_drone_state(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones);
int countElementOccurrences(const Drones *currentDrones);

#endif
