#include "expansion.h"
#include <stdbool.h>
#include <float.h>

#define PI 3.14159265358979323846
#define SQRT_3_OVER_2 0.86602540378 // sqrt(3) / 2

int isDuplicate(Target *objects, int size, float x, float y)
{
    for (int i = 0; i < size; i++)
    {
        if (objects[i].x == x && objects[i].y == y)
        {
            return 1;
        }
    }
    return 0;
}

// // generate targets
void generate_targets(Target *targets, int targets_size, float (*predefinedTargets)[2])
{
    if (predefinedTargets == NULL)
    {
        printf("Generate random targets \n");
        int n = targets_size * 0.50; // to detrmine
        int dir1, dir2;
        int mult1, mult2;
        for (int i = 0; i < targets_size; i++)
        {
            float x, y;
            do
            {
                dir1 = rand() % (6 + 1);
                dir2 = rand() % (6 + 1);
                mult1 = (rand() % (n + 1));
                mult2 = (rand() % (n + 1)); // Generate a random value between 0 and n (inclusive)
                x = mult1 * DIR_VECTORS[dir1][0];
                y = mult1 * DIR_VECTORS[dir1][1];

            } while ((abs(x) < 1000 && abs(y) < 1000) && isDuplicate(targets, i, x, y));

            targets[i].x = x;
            targets[i].y = y;
            targets[i].found = false;
        }
    }
    else
    {
        // Use predefined targets
        // Ensure that the predefinedTargets pointer is not NULL
        if (predefinedTargets != NULL)
        {
            for (int i = 0; i < targets_size; i++)
            { // Assuming there are 8 predefined targets
                targets[i].x = predefinedTargets[i][0] + 0.01;
                targets[i].y = predefinedTargets[i][1] + 0.01;
                targets[i].found = false;
            }
        }
    }
}

void save_targes(Target *targets, int targets_size, FILE *fp)
{
    for (int i = 0; i < targets_size; i++)
    {

        fprintf(fp, "(");
        fprintf(fp, "%d", 00);
        fprintf(fp, ", ");
        fprintf(fp, "%.6f", targets[i].x);
        fprintf(fp, ", ");
        fprintf(fp, "%.6f", targets[i].y);
        fprintf(fp, ", ");
        fprintf(fp, "%d", 10);
        fprintf(fp, ", ");
        fprintf(fp, "%d", 0);
        fprintf(fp, "),");
    }
    fprintf(fp, "\n");
}

bool isPointInsideHexagon(float pointX, float pointY, float centerX, float centerY)
{
    for (int i = 0; i < 6; ++i)
    {
        // Calculate angle for each vertex
        float angle = 2 * PI * i / 6;

        // Rotate point around the center
        float rotatedX = centerX + (pointX - centerX) * cos(angle) - (pointY - centerY) * sin(angle);
        float rotatedY = centerY + (pointX - centerX) * sin(angle) + (pointY - centerY) * cos(angle);

        // Check distance to the vertical line going through the hexagon center
        if (fabs(rotatedX - centerX) > effective_a * SQRT_3_OVER_2)
        {
            return false;
        }
    }
    return true;
}

// If many drones found the same target then keep only th eone with min id
void check_another_neighbor_detect_target(Drones drones[], struct Neighbors neighbors[], Drones *currentDrones, int numdrones)
{
    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[j][1]))
            {
                if (drones[i].targetfound && drones[i].id > currentDrones->id)
                    drones[i].targetfound = false;
            }
        }
    }
}

void set_state_target_check(Drones drones[], Drones *currentDrones, struct Neighbors neighbors[], Target *targets, int targets_size, int numdrones)
{
    // so when drone was border then will move many steps then border state will not be saved
    int previous_state = 0;
    for (int i = 0; i < targets_size; i++)
    {
        if (isPointInsideHexagon(targets[i].x, targets[i].y, currentDrones->x, currentDrones->y))
        {
            targets[i].found = true;
            currentDrones->targetfound = 1;
            int previous_state = currentDrones->state;

            if (currentDrones->state == Alone || currentDrones->state == Free)
            {
                currentDrones->state = Irremovable;
            }
            if (currentDrones->state == Border)
            {
                currentDrones->state = Irremovable_border;
            }
            if (currentDrones->state != previous_state)
                currentDrones->previous_state = previous_state;
        }
    }
}

// used to build ordered array so the closest targest to the sink will start building patht to the sink
int compareDrones(const void *d1, const void *d2)
{
    Drones *droneA = (Drones *)d1;
    Drones *droneB = (Drones *)d2;
    if (droneA->drone_distance < droneB->drone_distance)
        return -1;
    if (droneA->drone_distance > droneB->drone_distance)
        return 1;
    if (droneA->drone_distance == droneB->drone_distance)
    {
        if (droneA->id < droneB->id)
            return -1;
        else
            return 1;
    }
    return 0;
}

void findIrremovableDroneAround(Drones drones[], Drones *currentDrone, char result[MAX_SIZE][MAX_SIZE], int *irrmvble_id, int *resultSize, int numdrones)
{
    int count = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrone->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrone->y + DIR_VECTORS[j][1]) && (drones[i].state == Irremovable || drones[i].state == Irremovable_border))
            {
                char neighbors[3];
                sprintf(neighbors, "s%d", j);
                strcpy(result[count], neighbors);
                irrmvble_id[count] = drones[i].id;
                count++;
            }
        }
    }
    *resultSize = count;
}

void find_num_IrremovableDroneAround(Drones drones[], Drones *currentDrone, int *resultSize, int numdrones, int currentDrones_id, int sender_id)
{
    int count = 0;
    for (int j = 1; j < 7; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrone->x + DIR_VECTORS[j][0]) &&
                float_compare(drones[i].y, currentDrone->y + DIR_VECTORS[j][1]) &&
                (drones[i].state == Irremovable || drones[i].state == Irremovable_border) &&
                drones[i].id != sender_id &&
                drones[i].id != currentDrones_id)
            {
                count++;
            }
        }
    }
    *resultSize = count;
}

int has_irrmovable_drone_around_to_sink(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    int dir = 0;
    int min_distance_irrmovable = 0;
    int id = 0;
    int dir_to_check = 0;
    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[j][1]))
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    break;
                }
            }
        }
    }

    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[j][1]))
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0 && neighbors[drones[i].id].distances[0] <= min_distance_irrmovable)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    dir = j;
                }
            }
        }
    }
    return dir;
}

int has_irrmovable_drone_around_to_border(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    int dir = 0;
    int min_distance_irrmovable = 0;
    int id = 0;
    int dir_to_check = 0;
    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[j][1]))
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    break;
                }
            }
        }
    }

    for (int j = 1; j < neighbors[currentDrones->id].size; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[j][1]))
            {
                int irrmvbleSize2 = 0;
                find_num_IrremovableDroneAround(drones, &drones[i], &irrmvbleSize2, numdrones, currentDrones->id, sender_id);
                if (irrmvbleSize2 > 0 && neighbors[drones[i].id].distances[0] >= min_distance_irrmovable)
                {
                    min_distance_irrmovable = neighbors[drones[i].id].distances[0];
                    dir = j;
                }
            }
        }
    }
    return dir;
}

void findDirofSender(Drones drones[], Drones *currentDrone, char sender_dir[3], int numdrones, int sender_id)
{

    int count = 0;
    for (int j = 1; j < 7; j++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrone->x + DIR_VECTORS[j][0]) && float_compare(drones[i].y, currentDrone->y + DIR_VECTORS[j][1]) && (drones[i].id == sender_id))
            {
                sprintf(sender_dir, "s%d", j);
                break;
            }
        }
    }
}

int check_close_to_sink(Drones drones[], Drones *currentDrones, int numdrones, int dir)
{
    int closetosink_drone = 0;
    for (int i = 0; i < numdrones; i++)
    {
        // find what drone it is
        if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[dir][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[dir][1]) && drones[i].id_tag_to_sink != currentDrones->id_tag_to_sink)
        {
            if (abs(drones[i].x) < abs(currentDrones->x) && abs(drones[i].y) < abs(currentDrones->y))
            {
                closetosink_drone = 1;
            }
            break;
        }
    }
    return closetosink_drone;
}

float check_previous_border_distnace(Drones drones[], Drones *currentDrones, int numdrones, int dir)
{
    int previous_border = 0;
    float drone_distance;
    for (int i = 0; i < numdrones; i++)
    {
        // find what drone it is
        if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[dir][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[dir][1]))
        {
            if (((drones[i].previous_state == Border || drones[i].previous_state == Irremovable_border)) && drones[i].id_tag_to_sink != currentDrones->id_tag_to_sink)
            {
                return drones[i].drone_distance;
            }
        }
    }
    return -1; // if nothing found
}

// Find id of drone based on the direction from the current drone
int find_drone_id_around(Drones drones[], int current_drone_id, int numdrones, int dir)
{
    for (int i = 0; i < numdrones; i++)
    {
        // find what drone it is
        if (float_compare(drones[i].x, drones[current_drone_id].x + DIR_VECTORS[dir][0]) && float_compare(drones[i].y, drones[current_drone_id].y + DIR_VECTORS[dir][1]))
        {
            return drones[i].id;
        }
    }
    return -1;
}

int check_dir_close_to_goal(Drones drones[], Drones *currentDrones, int numdrones, int state)
{
    float min_distance_to_border = FLT_MAX;
    int desired_dir = -1;
    float min_difference = FLT_MAX;

    for (int dir = 1; dir < 7; dir++)
    {
        for (int i = 0; i < numdrones; i++)
        {
            if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[dir][0]) &&
                float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[dir][1]) &&
                drones[i].id_tag_to_sink != currentDrones->id_tag_to_sink)
            {

                if ((drones[i].state == state && (drones[i].previous_state != Border && drones[i].previous_state != Irremovable_border)))
                {
                    // Dont use distance to compare because sometimes 2 drones have same distance from sink but we need the distance between 2 points not with sink
                    float difference = sqrt(pow(drones[i].x - drones[currentDrones->closest_target].x, 2) + pow(drones[i].y - drones[currentDrones->closest_target].y, 2));

                    if (difference < min_difference)
                    {
                        // Update based on the smallest difference
                        min_difference = difference;
                        desired_dir = dir;
                    }
                }
                break; // the drone is found
            }
        }
    }
    return desired_dir;
}

bool build_path_to_sink(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    set_num_drones_at_neighbors(drones, &neighbors[currentDrones->id], currentDrones, numdrones);
    if ((currentDrones->x == 0 && currentDrones->y == 0) || (currentDrones->connect_sink)) // stop the recursion when arrive to sink or another path
    {
        return true;
    }

    int dir = 0;
    char irrmvble_dir[MAX_SIZE][MAX_SIZE];
    int irrmvble_id[6]; // all id  of irrmovable
    int irrmvbleSize = 0;
    setDist(&neighbors[currentDrones->id], currentDrones->x, currentDrones->y); // update for the next iteration                                                        // find the spots closer to the sink to decide how to set the priorities
    char closeSink[MAX_SIZE][MAX_SIZE];
    int closeSinkSize = 0;
    findMinDistances(&neighbors[currentDrones->id], closeSink, &closeSinkSize);
    findIrremovableDroneAround(drones, currentDrones, irrmvble_dir, irrmvble_id, &irrmvbleSize, numdrones); // it will accept only the one close to the sink
    // find the direction of the drone that send message to build the path
    char sender_dir[3];
    findDirofSender(drones, currentDrones, sender_dir, numdrones, sender_id);
    float min_irrmovable;

    // check if there is any irremovable drone in the neighbors and close to the sink
    if (irrmvbleSize > 0)
    {
        for (int k = 0; k < irrmvbleSize; k++)
        {
            if (irrmvble_id[k] != sender_id && drones[irrmvble_id[k]].id_tag_to_sink != currentDrones->id_tag_to_sink) // then after it will examined if it is connected to sink or not
            {
                min_irrmovable = neighbors[irrmvble_id[k]].distances[0];
                sscanf(irrmvble_dir[k], "s%d", &dir);
                break;
            }
        }
    }

    // Find a previous border that is close to the target want to connect to
    if (dir == 0)
    {
        float min_distance_difference = FLT_MAX; // Variable to track the minimum difference

        for (int j = 1; j < neighbors[currentDrones->id].size; j++) // neighbors[currentDrones->id].size=7
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0)
            {

                float neig_border_distance = check_previous_border_distnace(drones, currentDrones, numdrones, j);
                if (neig_border_distance != -1)
                {
                    int neig_drone_id = find_drone_id_around(drones, currentDrones->id, numdrones, j);
                    float difference = sqrt(pow(drones[neig_drone_id].x - drones[currentDrones->closest_target].x, 2) + pow(drones[neig_drone_id].y - drones[currentDrones->closest_target].y, 2));
                    if (difference < min_distance_difference)
                    {
                        min_distance_difference = difference;
                        dir = j;
                    }
                }
            }
        }
    }

    // No irreomvabe from another path found , then try to find the closest free drone around
    if (dir == 0)
    {
        int desired_dir = check_dir_close_to_goal(drones, currentDrones, numdrones, Free);
        if (desired_dir != -1)
            dir = desired_dir;
    }
    // No Irremovable, free or previous border then try to find border close to conect with which lead to another drone to sink
    if (dir == 0)
    {
        int desired_dir = check_dir_close_to_goal(drones, currentDrones, numdrones, Border);
        if (desired_dir != -1)
            dir = desired_dir;
    }

    // No drones found, not previous border and not irremovable not free not border, return and wait to next itration to connect wwith the sink
    if (dir == 0)
    {
        return false;
    }

    for (int i = 0; i < numdrones; i++)
    {
        if (float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[dir][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[dir][1]))
        {
            if (drones[i].state == Border) // it is border then make it border and irrmovable
            {
                drones[i].state = Irremovable_border;
                currentDrones->previous_state = Irremovable;
                drones[i].id_tag_to_sink = currentDrones->id_tag_to_sink;
                drones[i].closest_target = currentDrones->closest_target;
                // you should go to the sink
                return build_path_to_sink(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
            else if (drones[i].state == Irremovable || drones[i].state == Irremovable_border)
            {
                // Arrived to Irremovable should check if it belongds to connected to sink path
                // if it is not connected then consider it part of the current target and keep searching to connect with the sink
                // and this drone should try to connect to the other target that is the aime
                // and that is necessary because otherwise it will try to connect to a path very far, and this drone should be continuation of the one current drone building
                if (!drones[i].connect_sink)
                {
                    drones[i].id_tag_to_sink = currentDrones->id_tag_to_sink;
                    drones[i].closest_target = currentDrones->closest_target;
                    return build_path_to_sink(neighbors, &drones[i], drones, numdrones, currentDrones->id); // arrived to drone with irrmovable state
                }
                else
                    return true; // Drone is connect to sink then retuen ( the path found )
            }
            else
            {
                drones[i].state = Irremovable;
                drones[i].previous_state = Irremovable;
                drones[i].id_tag_to_sink = currentDrones->id_tag_to_sink;
                drones[i].closest_target = currentDrones->closest_target;
                return build_path_to_sink(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
        }
    }
}

/*
here you should connect to the farest and close to the border or irrmovable
*/
// this should be doen until arriving to a drone with a state border
int build_path_to_border(struct Neighbors neighbors[], Drones *currentDrones, Drones drones[], int numdrones, int sender_id)
{
    // printf("curre_drone %d \n", currentDrones->id);
    set_num_drones_at_neighbors(drones, &neighbors[currentDrones->id], currentDrones, numdrones);
    // no need to do so if the drone is irrmovable and border
    if (currentDrones->state == Irremovable_border || currentDrones->state == Border) //|| currentDrones->previous_state == 3) // stop the recursion when arrive to border
    {
        return currentDrones->id;
    }
    int dir = 0;
    char irrmvble_dir[MAX_SIZE][MAX_SIZE];
    int irrmvble_id[6]; // all id  of irrmovable
    int irrmvbleSize = 0;
    // it will accept only the one close to the sink
    setDist(&neighbors[currentDrones->id], currentDrones->x, currentDrones->y); // find the spots distances

    char closeBorder[MAX_SIZE][MAX_SIZE];
    int closeBorderSize;
    findMaxDistances(&neighbors[currentDrones->id], closeBorder, &closeBorderSize); // find spot that is far from the sink ( towards the border)
    char sender_dir[3];
    findIrremovableDroneAround(drones, currentDrones, irrmvble_dir, irrmvble_id, &irrmvbleSize, numdrones); // it will accept only the one close to the sink
    //    find the direction of the drone that send message to build the path
    findDirofSender(drones, currentDrones, sender_dir, numdrones, sender_id);

    for (int k = 0; k < irrmvbleSize; k++)
    {
        if (irrmvble_id[k] != sender_id && drones[irrmvble_id[k]].state == Irremovable_border)
        {
            sscanf(irrmvble_dir[k], "s%d", &dir);
            break;
        }
    }

    // No irremovable found try to connect to any close drone twards the border
    if (dir == 0)
    {
        float max_dist = 0;
        for (int j = 1; j < neighbors[currentDrones->id].size; j++)
        {

            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0 &&
                neighbors[currentDrones->id].distances[0] <= neighbors[currentDrones->id].distances[j])
            {

                max_dist = neighbors[currentDrones->id].distances[j];
                dir = j; // should be assigned beause it is possible to dont find others
                break;
            }
        }
        // It is possible to have many drones has the same distance so it can be chosen randomly
        for (int j = 1; j < neighbors[currentDrones->id].size; j++)
        {
            if (strcmp(neighbors[currentDrones->id].Status[j], "o") == 0 &&
                strcmp(neighbors[currentDrones->id].keys[j], sender_dir) != 0)
            {
                if (float_compare(max_dist, neighbors[currentDrones->id].distances[j]))
                {
                    if (rand() % 2) // This results in 0 or 1 randomly
                        dir = j;    // No need to update max_dist since it's equal
                }
                else if (max_dist < neighbors[currentDrones->id].distances[j])
                {
                    max_dist = neighbors[currentDrones->id].distances[j];
                    dir = j;
                }
            }
        }
    }
    // nothing found return -1
    if (dir == 0)
    {
        return currentDrones->id;
    }
    //  same in real life after reciving a message should check the state of the drone if it is not border it should forward the message
    for (int i = 0; i < numdrones; i++)
    {
        if ((float_compare(drones[i].x, currentDrones->x + DIR_VECTORS[dir][0]) && float_compare(drones[i].y, currentDrones->y + DIR_VECTORS[dir][1])) && dir != 0)
        {
            if (drones[i].state == Border) // it is border then make it border and irrmovable
            {
                drones[i].state = Irremovable_border;
                drones[i].id_tag_to_border = currentDrones->id_tag_to_border;
                drones[i].id_tag_to_sink = currentDrones->id_tag_to_sink;
                return build_path_to_border(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
            else if (drones[i].state == Irremovable_border)
            {
                return drones[i].id; // arrived to drone with irrmovable state
            }
            else
            {
                drones[i].state = Irremovable;
                drones[i].id_tag_to_border = currentDrones->id_tag_to_border;
                drones[i].id_tag_to_sink = currentDrones->id_tag_to_sink;
                return build_path_to_border(neighbors, &drones[i], drones, numdrones, currentDrones->id);
            }
        }
    }
}

/*
- Find the drones that found the targets and  need to build path to the sink and to the border
- Also add the drone that belongs to sink connected to the border in case no target found in previous spanning so it can be connected to it
- Arrange the drones in targets_order array in ascending order so the ones closest to sink start building the path
*/
void perform_spanning(Drones drones[], struct Neighbors DroneNeighbors[], int numdrones, FILE *fp)
{
    Drones validDrones[numdrones];
    int validCount = 0;

    /*
    In the first spanning each drone can connect to the closest target if it is already built a path  or to the sink itself if it is close to it
    Actually in this way the closest to the sink will connect to it and the others targets willl try to connect to the on very close to sink
    */
    /*
    Note that drones[i].id_tag_to_border = -1; because this process is continous not like building path to sink
    building path to border should be updated after each spannning to connect to the new border
     */
    for (int i = 0; i < numdrones; i++)
    {
        if ((drones[i].state == Irremovable || drones[i].state == Irremovable_border) && drones[i].targetfound)
        {
            validDrones[validCount++] = drones[i];
            drones[i].id_tag_to_border = -1;
        }
        if (drones[i].x == 0 && drones[i].y == 0)
        {
            validDrones[validCount++] = drones[i]; // add the sink so there is possibility to connect to it directly
        }
    }

    // Sort the valid drones by distance.
    qsort(validDrones, validCount, sizeof(Drones), compareDrones);

    // Allocate memory for targets_order based on valid drone count.
    int *targets_order = (int *)malloc(validCount * sizeof(int));
    for (int i = 0; i < validCount; i++)
    {
        targets_order[i] = validDrones[i].id;
    }

    // The closest to sink starts and so on until all build path
    for (int i = 0; i < validCount; i++)
    {
        float mind_difference = FLT_MAX;
        int target_close_id = -1;

        for (int j = 0; j < validCount; j++)
        {
            /* drones[targets_order[j]].drone_distance < drones[targets_order[i]].drone_distance this condition is necessary to
            garentee that the drone will connect to it already built it is path
            since the one with smallest distanc will start first*/
            if (targets_order[j] != targets_order[i] && drones[targets_order[j]].drone_distance < drones[targets_order[i]].drone_distance)
            {
                float difference = sqrt(pow(drones[targets_order[j]].x - drones[targets_order[i]].x, 2) + pow(drones[targets_order[j]].y - drones[targets_order[i]].y, 2));
                if (difference <= mind_difference)
                {
                    mind_difference = difference;
                    target_close_id = drones[targets_order[j]].id;
                }
            }
        }

        drones[targets_order[i]].closest_target = target_close_id;
        drones[targets_order[i]].id_tag_to_sink = drones[targets_order[i]].id; // set the tag of the path equal to the id of target that try to built it
        bool connected = false;
        // start the recursive function to build the path and each drone on that path will be taged ith the id of the target that constructing the path
        set_num_drones_at_neighbors(drones, &DroneNeighbors[targets_order[i]], &drones[targets_order[i]], numdrones);
        connected = build_path_to_sink(DroneNeighbors, &drones[targets_order[i]], drones, numdrones, drones[targets_order[i]].id);
        // When arriving to the sink or to another irremovable drone that belongs to another path that is connected to the sink
        // stop and mark all the drones that are taged with the same id of target with cconnected to recognize that they are connected to sink

        /*
         Build path to the border performed by all the target except the sink that is in targets_order , because the sink will be at the first index
         and it will execute build path to border, which should be done only if no target id is found ( so no other path was built)
         */
        if (drones[targets_order[i]].drone_distance != 0) // not sink
        {
            drones[targets_order[i]].id_tag_to_border = drones[targets_order[i]].id;
            set_num_drones_at_neighbors(drones, &DroneNeighbors[targets_order[i]], &drones[targets_order[i]], numdrones);
            int id_irr_border = build_path_to_border(DroneNeighbors, &drones[targets_order[i]], drones, numdrones, drones[targets_order[i]].id);
            if (id_irr_border >= 0)
            {
                drones[i].id_border_connection = id_irr_border;
            }
        }
        // tag all those drones as connected to the sink so they are able to connect with them
        // remember that during building path to border all those border thier id_tag_to_sink was also taged with the id of triggering drone
        if (connected)
        {
            for (int j = 0; j < numdrones; j++)
            {
                if (drones[j].id_tag_to_sink == drones[targets_order[i]].id_tag_to_sink)
                {
                    drones[j].connect_sink = true;
                }
            }
        }
    }
    /*
    Since the sink was included in the array, then if no other irremovable
    drone arround belong to another path ( no target found) from previous building path to sink then it will try to build its own path to border
    */
    for (int i = 0; i < validCount; i++)
    {
        if (drones[targets_order[i]].drone_distance == 0) // sink ( notice that sink can be an id not only 0 ), thus distance used
        {
            // there is only the sink in the array that means no target is found, so a path from sink is formed
            if (validCount == 1)
            {
                drones[targets_order[i]].id_tag_to_border = drones[targets_order[i]].id;
                set_num_drones_at_neighbors(drones, &DroneNeighbors[targets_order[i]], &drones[targets_order[i]], numdrones);
                int id_irr_border = build_path_to_border(DroneNeighbors, &drones[targets_order[i]], drones, numdrones, drones[targets_order[i]].id);
                if (id_irr_border >= 0)
                {
                    drones[targets_order[i]].id_border_connection = id_irr_border;
                    // tag all those drones as connected to the sink so they are able to connect with them
                    // remember that during building path to border all those border thier id_tag_to_sink was also taged with the id of triggering drone
                    for (int j = 0; j < numdrones; j++)
                    {
                        if (drones[j].id_tag_to_sink == drones[targets_order[i]].id_tag_to_sink)
                        {
                            drones[j].connect_sink = true;
                        }
                    }
                    break;
                }
            }
            else
            {
                // in case thi path is not formed due to the existance of target then no need to add the end of this path to next iteration
                drones[targets_order[i]].id_border_connection = -1;
            }
        }
    }

    free(targets_order);

    saveDrones(drones, numdrones, fp);
}

void perform_further_spanning(Drones drones[], struct Neighbors DroneNeighbors[], int numdrones, FILE *fp)
{
    Drones validDrones[numdrones];
    int validCount = 0;

    for (int i = 0; i < numdrones; i++)
    {
        if ((drones[i].state == Irremovable || drones[i].state == Irremovable_border) && drones[i].targetfound)
        {
            validDrones[validCount++] = drones[i];
            drones[i].id_tag_to_border = -1;
        }
        if ((drones[i].x == 0 && drones[i].y == 0) && drones[i].id_border_connection != -1)
        {
            validDrones[validCount++] = drones[drones[i].id_border_connection]; // add the border of the sink if it is there
        }
    }

    qsort(validDrones, validCount, sizeof(Drones), compareDrones);

    int *targets_order = (int *)malloc(validCount * sizeof(int));
    for (int i = 0; i < validCount; i++)
    {
        targets_order[i] = validDrones[i].id;
    }

    for (int i = 0; i < validCount; i++)
    {
        float mind_difference = FLT_MAX;
        int target_close_id = -1;

        // If target drone is not connected to the sink then try to connect it
        if (!drones[targets_order[i]].connect_sink)
        {
            for (int j = 0; j < validCount; j++)
            {
                if (targets_order[j] != targets_order[i] && drones[targets_order[j]].drone_distance < drones[targets_order[i]].drone_distance)
                {
                    float difference = sqrt(pow(drones[targets_order[j]].x - drones[targets_order[i]].x, 2) + pow(drones[targets_order[j]].y - drones[targets_order[i]].y, 2));
                    if (difference < mind_difference)
                    {
                        mind_difference = difference;
                        target_close_id = drones[targets_order[j]].id;
                    }
                }
            }
        }
        drones[targets_order[i]].closest_target = target_close_id;
        drones[targets_order[i]].id_tag_to_sink = drones[targets_order[i]].id;
        bool connected = false;

        set_num_drones_at_neighbors(drones, &DroneNeighbors[targets_order[i]], &drones[targets_order[i]], numdrones);
        connected = build_path_to_sink(DroneNeighbors, &drones[targets_order[i]], drones, numdrones, drones[targets_order[i]].id);

        /*
        Build path to the border from the most close connect path to the sink.
        and since the drone that is connect sink to border from the last phase was included in the array then if no other irremovable
        drone arround belong to another path it will try to build its own path to new border
        */
        drones[targets_order[i]].id_tag_to_border = drones[targets_order[i]].id;
        int id_irr_border = build_path_to_border(DroneNeighbors, &drones[targets_order[i]], drones, numdrones, drones[targets_order[i]].id);
        if (id_irr_border >= 0)
        {
            drones[targets_order[i]].id_border_connection = id_irr_border;
        }
        if (connected)
        {
            for (int j = 0; j < numdrones; j++)
            {
                if (drones[j].id_tag_to_sink == drones[targets_order[i]].id_tag_to_sink)
                {
                    drones[j].connect_sink = true;
                }
            }
        }
    }
    free(targets_order);
    saveDrones(drones, numdrones, fp);
}
