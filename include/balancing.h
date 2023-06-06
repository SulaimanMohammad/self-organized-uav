#ifndef BALANCING_H
#define BALANCING_H
#include "expansion.h"

int count_border_drones_AtPosition(Drones drones[], int numdrones, float x, float y);
void set_num_drones_border_at_neighbors(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones);
int dir_minimum_drones_in_border_neigboor(struct Neighbors *neighbors);
int border_drone_with_min_drones(struct Neighbors *neighbors, Drones *currentDrones, Drones drones[], int numdrones, int dir, int *arrived_to_border);
void move_free_until_border(struct Neighbors *neighbors, Drones drones[], Drones *currentDrones, int numdrones);
#endif