#ifndef EXPANSION_H
#define EXPANSION_H
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

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
    int id;
    float x;
    float y;
    int state;
    int targetfound;
    // alone=0 free=1 border=2 irrmovable=3 // irrovable and border= 4
    int previous_state;
    // when the border s formed
    // if a drone become irrmovable then the dominated direction  will not change
    int allowed_neighborsSize;
    int *allowed_to_goto; // Now an int array;

} Drones;

struct Neighbors
{
    char keys[MAX_SIZE][MAX_SIZE];
    float distances[MAX_SIZE];
    float spot_priority[MAX_SIZE];
    char Status[MAX_SIZE][MAX_SIZE]; // occupied or free
    int size;
    int spot_num_drones[MAX_SIZE]; // you needed to be incremented when a new arriver
};

typedef struct Target
{
    float x;
    float y;
    bool found;
} Target;
int float_compare(float num1, float num2);
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
void findMaxDistances(struct Neighbors *neighbors, char result[MAX_SIZE][MAX_SIZE], int *resultSize);

void findPriority(struct Neighbors *neighbors, char result[MAX_SIZE][MAX_SIZE], int *resultSize);

// generate random number [A,B[ , b is not included
float randomFloat(float A, float B);

int randomInt(int A, int B);

void setPriorities(struct Neighbors *neighbors);
void findBorderDroneAround(Drones drones[], Drones *currentDrone, char result[MAX_SIZE][MAX_SIZE], int *resultSize, int numdrones);
void allowed_neigbors_for_further_expan(Drones drones[], Drones *currentDrone, char allowed_neighbors[MAX_SIZE][MAX_SIZE], int *allowed_neighborsSize, int numdrones);
void setPriorities_further_expan(Drones drones[], Drones *currentDrone, struct Neighbors *neighbors, int numdrones);
void initializeDrones(Drones drones[], int numdrones);
void free_memory_drone(Drones drones[], int numdrones);

void printDrones(Drones drones[], int numdrones);
void saveDrones(Drones drones[], int numdrones, FILE *fp);

int countdronesAtPosition(Drones drones[], int numdrones, float x, float y);
int countdronesAtPosition_with_specific_state(Drones drones[], int numdrones, float x, float y, int states);
void moveDrones(Drones *Drones, enum Direction dir);

void set_num_drones_at_neighbors(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones);
void check_drone_spot(Drones drones[], Drones *currentDrones, int numdrones);
void check_drone_spot_further_expan(Drones drones[], Drones *currentDrones, int numdrones);

void find_border_update_drone_state(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones);
int countElementOccurrences(const Drones *currentDrones);
int target_in_area(Drones *currentDrones, Target *targets, int targets_num, int distance, int length);
void perform_first_expansion(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, FILE *fp);
void perform_further_expansion(Drones *drones, struct Neighbors *DroneNeighbors, int numdrones, FILE *fp);
void form_border_and_update_states(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, Target targets[], int targets_size, FILE *fp);
void form_further_border_and_update_states(Drones *drones, struct Neighbors DroneNeighbors[], int numdrones, Target targets[], int targets_size, FILE *fp);
#endif
