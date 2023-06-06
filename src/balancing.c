#include "expansion.h"

// count number of drones in a position but only if the drone is in border state
int count_border_drones_AtPosition(Drones drones[], int numdrones, float x, float y)
{
    int count = 0;
    int is_there_border_drone = 0;
    for (int i = 0; i < numdrones; i++)
    {
        // should count all the drones at the neigboor
        // since that contains neigboor not part of border it will be evited
        {
            if (drones[i].x == x && drones[i].y == y)
                count++;
        }
        if (drones[i].x == x && drones[i].y == y && (drones[i].state == 2 || drones[i].state == 4))
        {
            is_there_border_drone++;
        }
    }
    if (is_there_border_drone == 0) // no border drone foun at the neigbors so it should not be considered
        count = -1;

    return count;
}

void set_num_drones_border_at_neighbors(Drones drones[], struct Neighbors *neighbors, Drones *currentDrones, int numdrones)
{
    int count_drons = 0;
    for (int j = 0; j < 7; j++) // need to check the number in each neigboor and add to w in neigboor of the Drones
    {

        count_drons = count_border_drones_AtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
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

int dir_minimum_drones_in_border_neigboor(struct Neighbors *neighbors)
{
    char *spot;
    int count;
    spot = neighbors->keys[0];
    count = neighbors->w[0];
    // the number of drone in each neigboor stored in variable "w"
    for (int i = 0; i < neighbors->size; i++)
    {                                                       // need to check the spot, and the drone there if it is border because the neigboor can contain non-border drone
        if (neighbors->w[i] > 0 && neighbors->w[i] < count) // neighbors->w >0 means it is border ( see count_border_drones_AtPosition )
        {
            count = neighbors->w[i];
            spot = neighbors->keys[i]; // save the name of the spot that has minimum number
        }
    }

    int dir;
    sscanf(spot, "s%d", &dir); // extract the drection number from "sn"
    return dir;
}

int border_drone_with_min_drones(struct Neighbors *neighbors, Drones *currentDrones, Drones drones[], int numdrones, int dir, int *arrived_to_border)
{
    int direction_to_go = 0; // stay in same place
    for (int i = 0; i < numdrones; i++)
    {                                                                                                                // since it is for loop will iterate in all the drone but should not consider the drone we are working on
        if (drones[i].x == currentDrones->x && drones[i].y == currentDrones->y && drones[i].id != currentDrones->id) // check if the drone there is already border
        {
            if (drones[i].state == 2 || drones[i].state == 4) // check the drone that share the same spot if it is border
            {
                set_num_drones_border_at_neighbors(drones, neighbors, &drones[i], numdrones);
                direction_to_go = dir_minimum_drones_in_border_neigboor(neighbors);
                *arrived_to_border = 1;
            }
        }
    }
    // notice if the dir doesnt lead yet to border then this fuction will return 0 so the drone will stay and move next iteration
    // if the next step will lead to the one of the nigboor that where number of droen are less then it will have valuze
    return direction_to_go;
}

// the drone will try to move until it arrives to the border which is indecated by arrived_to_border
// in each step the drone check if it is arrived to the border if not it will continut to move far from sink towards the border
// notice border_drone_with_min_drones will return 0 means no move if the drone is not at the border
// the moment the drones arrive to the border it will start to check the neigbors of the drone and find the spot that is border and contains the min of drones
// then border_drone_with_min_drones will return the direction to go the minimum and makes direction_border_neighbor=1
void move_free_until_border(struct Neighbors *neighbors, Drones drones[], Drones *currentDrones, int numdrones)
{
    int dir = 0;
    int arrived_to_border = 0;
    while (arrived_to_border != 1)
    {
        char closeBorder[MAX_SIZE][MAX_SIZE];
        int closeBorderSize;
        int direction_border_neighbor;
        check_drone_spot(drones, currentDrones, numdrones);     // check if the drone is alone or nots
        setDist(neighbors, currentDrones->x, currentDrones->y); // update for the next iteration
        // no need to check the num_drones_at_neighbors or to find priority because we need to go the border not to couver
        findMaxDistances(neighbors, closeBorder, &closeBorderSize); // find spot that is far from the sink ( towards the border)
        sscanf(closeBorder[0], "s%d", &dir);
        moveDrones(currentDrones, dir);
        append_new_step(currentDrones, dir);
        direction_border_neighbor = border_drone_with_min_drones(neighbors, currentDrones, drones, numdrones, dir, &arrived_to_border); // check if the found drone is border so stop moving if not no
        moveDrones(currentDrones, direction_border_neighbor);
        if (direction_border_neighbor != 0) // if the drone did not arrive to the border then it border_drone_with_min_drones will return zero dont save it in the movement list
            append_new_step(currentDrones, direction_border_neighbor);
        setDist(neighbors, currentDrones->x, currentDrones->y); // update for the next iteration
    }
}
