#ifndef SPANNING_H
#define SPANNING_H
#include "expansion.h"
int isDuplicate(Target *objects, int size, float x, float y);
void generate_targets(Target *targets, int targets_size, float (*predefinedTargets)[2]);
void save_targes(Target *targets, int targets_size, FILE *fp);
void set_state_target_check(Drones drones[], Drones *currentDrones, struct Neighbors neighbors[], Target *targets, int targets_size, int numdrones);
void findIrremovableDroneAround(Drones drones[], Drones *currentDrone, char result[MAX_SIZE][MAX_SIZE], int *irrmvble_id, int *resultSize, int numdrones);
void find_num_IrremovableDroneAround(Drones drones[], Drones *currentDrone, int *resultSize, int numdrones, int currentDrones_id, int sender_id);
int has_irrmovable_drone_around_to_sink(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id);
int has_irrmovable_drone_around_to_border(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id);

void findDirofSender(Drones drones[], Drones *currentDrone, char sender_dir[3], int numdrones, int sender_id);
int check_close_to_sink(Drones drones[], Drones *currentDrones, int numdrones, int dir);
bool build_path_to_sink(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id);
int build_path_to_border(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id);
void perform_spanning(Drones drones[], struct Neighbors DroneNeighbors[], int numdrones, FILE *fp);
void perform_further_spanning(Drones drones[], struct Neighbors DroneNeighbors[], int numdrones, FILE *fp);
#endif