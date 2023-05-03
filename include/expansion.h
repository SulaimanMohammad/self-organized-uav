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
extern double DIR_VECTORS[7][2];

typedef struct Point
{
    float x;
    float y;
    char stat; // free , border , irrmovable
} Point;

struct Dictionary
{
    char keys[MAX_SIZE][MAX_SIZE];
    float distances[MAX_SIZE];
    float v[MAX_SIZE];
    char Status[MAX_SIZE][MAX_SIZE]; // occupied or free
    int size;
    int w[MAX_SIZE]; // you needed to be incremented when a new arriver
};

// Initialize the dictionary
void initDictionary(struct Dictionary *dict);

// Add a new key-value pair to the dictionary
void addEntry(struct Dictionary *dict, char key[], float value);

// this create the 6 spots with coordiantes regaring the sink not with the respect of the drone position
void creatSpots(struct Dictionary *dict, const float Dx, const float Dy);

void setStatus(struct Dictionary *dict, char key[], char state[], int W);

// Get the value associated with a key in the dictionary
float getDist(struct Dictionary *dict, char key[]);

float getPriority(struct Dictionary *dict, char key[]);

char *getStatus(struct Dictionary *dict, char key[]);

// return the key of object in the minimum distance
void findMinDistances(struct Dictionary *dict, char result[MAX_SIZE][MAX_SIZE], int *resultSize);

void findPriority(struct Dictionary *dict, char result[MAX_SIZE][MAX_SIZE], int *resultSize);

// generate random number [A,B[ , b is not included
float randomFloat(float A, float B);

int randomInt(int A, int B);

void setPriorities(struct Dictionary *dict);

void initializePoints(Point points[], int numPoints);

void printPoints(Point points[], int numPoints);

int countPointsAtPosition(Point points[], int numPoints, float x, float y);

void movePoint(Point *point, enum Direction dir);

void set_num_drones_at_neighbors(Point points[], struct Dictionary *dict, Point *currentPoint, int numPoints);

#endif
