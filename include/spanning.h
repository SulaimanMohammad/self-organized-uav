#ifndef SPANNING_H
#define SPANNING_H
#include "expansion.h"
void generate_random_targets(Target *targets, int targets_size);
void save_targes(Target *targets, int targets_size, FILE *fp);
void set_state_target_check(Drones *currentDrones, Target *targets, int targets_size);
void build_path_to_sink(struct Neighbors *neighbors, Drones *currentDrones, Drones drones[], int numdrones);
void build_path_to_border(struct Neighbors *neighbors, Drones *currentDrones, Drones drones[], int numdrones);
#endif