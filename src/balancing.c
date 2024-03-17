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
            if (float_compare(drones[i].x, x) && float_compare(drones[i].y, y))
                count++;
        }
        if (float_compare(drones[i].x, x) && float_compare(drones[i].y, y) && (drones[i].state == Border || drones[i].state == Irremovable_border))
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
    for (int j = 0; j < 7; j++) // need to check the number in each neigboor and add to spot_num_drones in neigboor of the Drones
    {

        count_drons = count_border_drones_AtPosition(drones, numdrones, currentDrones->x + DIR_VECTORS[j][0], currentDrones->y + DIR_VECTORS[j][1]);
        if (count_drons > 0)
        {
            setStatus(neighbors, neighbors->keys[j], "o", count_drons); // neighborsionaries[i].spot_num_drones[j]
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
    /*the resason of puting -1 because where the drone will arive to border then it will count itself also so that means 2 in the sopt
    so the the check should be based on the number of drones on the niegbor out of counting the drone do that
    the reason it is important , for the path of the expasnsion
    suppose a drone arrived to a spot of border then withut using -1 then it will use to other border is occupied by the drone of the border because the count will =1 while where the drone is=2
    and that will not help because it would effect on the path of expansion if it was not necesssary
    */
    count = neighbors->spot_num_drones[0] - 1;
    // the number of drone in each neigboor stored in variable "spot_num_drones"
    for (int i = 0; i < neighbors->size; i++)
    {                                                                                   // need to check the spot, and the drone there if it is border because the neigboor can contain non-border drone
        if (neighbors->spot_num_drones[i] > 0 && neighbors->spot_num_drones[i] < count) // neighbors->spot_num_drones >0 means it is border ( see count_border_drones_AtPosition )
        {
            count = neighbors->spot_num_drones[i];
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
    { // since it is for loop will iterate in all the drone but should not consider the drone we are working on

        if (float_compare(drones[i].x, currentDrones->x) && float_compare(drones[i].y, currentDrones->y) && drones[i].id != currentDrones->id) // check if the drone there is already border
        {

            if (drones[i].state == Border || drones[i].state == Irremovable_border) // check the drone that share the same spot if it is border
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
        // check if the drone is alone or nots ( no need to check for being alone because that start with free after spanning and expansion )
        // and also the drone will go to somewhere where it will stay on the border so it wll not be alone
        setDist(neighbors, currentDrones->x, currentDrones->y); // update for the next iteration
        // no need to check the num_drones_at_neighbors or to find priority because we need to go the border not to couver
        findMaxDistances(neighbors, closeBorder, &closeBorderSize); // find spot that is far from the sink ( towards the border)
        sscanf(closeBorder[0], "s%d", &dir);
        moveDrones(currentDrones, dir);
        // printf("drone %d moves to spot %d\n", currentDrones->id, dir);
        direction_border_neighbor = border_drone_with_min_drones(neighbors, currentDrones, drones, numdrones, dir, &arrived_to_border); // check if the found drone is border so stop moving if not no
        moveDrones(currentDrones, direction_border_neighbor);
        setDist(neighbors, currentDrones->x, currentDrones->y); // update for the next iteration
    }
}

void perform_balancing_phase(Drones drones[], struct Neighbors DroneNeighbors[], int numdrones, FILE *fp)
{
    int border_drone_matrix[numdrones];
    int size_border_drone_matrix = 0;

    for (int i = 0; i < numdrones; i++)
    {

        if ((drones[i].state == Border) || (drones[i].state == Irremovable_border))
        {
            border_drone_matrix[size_border_drone_matrix] = i;
            size_border_drone_matrix++;
            int count_drons = 0;
            for (int j = 0; j < 7; j++)
            {
                count_drons = countdronesAtPosition(drones, numdrones, drones[i].x + DIR_VECTORS[j][0], drones[i].y + DIR_VECTORS[j][1]);
                if (count_drons > 0)
                {
                    // Find the index of 'j' in 'allowed_to_goto'
                    int remove_index = -1;
                    for (int k = 0; k < drones[i].allowed_neighborsSize; k++)
                    {
                        if (drones[i].allowed_to_goto[k] == j)
                        {
                            remove_index = k;
                            break;
                        }
                    }

                    // If found, shift elements to the left to remove the item
                    if (remove_index != -1)
                    {
                        for (int k = remove_index; k < drones[i].allowed_neighborsSize - 1; k++)
                        {
                            drones[i].allowed_to_goto[k] = drones[i].allowed_to_goto[k + 1];
                        }

                        // Decrement the size of 'allowed_to_goto'
                        drones[i].allowed_neighborsSize--;
                    }
                }
            }
        }
    }
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state == Free) // drone is free
        {
            move_free_until_border(&DroneNeighbors[i], drones, &drones[i], numdrones);
        }
    }
    for (int i = 0; i < numdrones; i++)
    {
        if (drones[i].state == 1)
        {
            for (int j = 0; j < size_border_drone_matrix; j++)
            {
                int border_drone_num = border_drone_matrix[j];
                if (((drones[border_drone_num].state == Border) || (drones[border_drone_num].state == Irremovable_border)) &&
                    float_compare(drones[i].x, drones[border_drone_num].x) && float_compare(drones[i].y, drones[border_drone_num].y))

                {
                    int copySize = drones[border_drone_num].allowed_neighborsSize;
                    for (int k = 0; k < copySize; k++)
                    {
                        drones[i].allowed_to_goto[k] = drones[border_drone_num].allowed_to_goto[k];
                    }

                    // If drones[border_drone_num].allowed_to_goto is smaller, fill the rest with zeros
                    for (int k = copySize; k < 7; k++)
                    {
                        drones[i].allowed_to_goto[k] = 0; // Fill with zeros
                    }
                    drones[i].allowed_neighborsSize = copySize;
                }
            }
        }
    }

    saveDrones(drones, numdrones, fp);
}
