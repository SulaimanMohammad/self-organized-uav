#include "expansion.h"
#include "spanning.h"
#include "balancing.h"
#include <limits.h>
#include <stdio.h>

#define MAX_LINE_LENGTH 10000

// Function to count the number of num_poi blocks in the file
int count_num_poi(FILE *file)
{
    char line[MAX_LINE_LENGTH];
    int count = 0;

    while (fgets(line, sizeof(line), file))
    {
        int num_poi;
        if (sscanf(line, "num_poi: %d", &num_poi) == 1) // Match a line that contains "num_poi: <number>"
        {
            count++;
        }
    }
    rewind(file);
    return count;
}

int ****read_coordinates(FILE *file, int *num_poi_array, int size_set_of_target, int *block_count)
{
    char line[MAX_LINE_LENGTH];
    int num_poi = 0, current_block = -1, line_in_block = 0;

    *block_count = count_num_poi(file);
    int ****blocks = malloc(*block_count * sizeof(int ***));
    if (blocks == NULL)
    {
        perror("Memory allocation failed for blocks");
        exit(EXIT_FAILURE);
    }

    // Read the file line by line
    while (fgets(line, sizeof(line), file))
    {

        // Check for the start of a new block
        if (sscanf(line, "num_poi: %d", &num_poi) == 1)
        {

            current_block++;
            line_in_block = 0;
            num_poi_array[current_block] = num_poi;
            if (num_poi == 0)
            {
                blocks[current_block] = NULL; // No points to allocate
                continue;
            }

            // Allocate space for each block based on num_poi
            blocks[current_block] = malloc(size_set_of_target * sizeof(int **));
            if (blocks[current_block] == NULL)
            {
                perror("Memory allocation failed for blocks[current_block]");
                exit(EXIT_FAILURE);
            }

            for (int i = 0; i < size_set_of_target; i++)
            {
                blocks[current_block][i] = malloc(num_poi * sizeof(int *));
                if (blocks[current_block][i] == NULL)
                {
                    perror("Memory allocation failed for blocks[current_block][i]");
                    exit(EXIT_FAILURE);
                }

                for (int j = 0; j < num_poi; j++)
                {
                    blocks[current_block][i][j] = malloc(2 * sizeof(int));
                    if (blocks[current_block][i][j] == NULL)
                    {
                        perror("Memory allocation failed for blocks[current_block][i][j]");
                        exit(EXIT_FAILURE);
                    }
                }
            }
        }
        else
        {
            if (current_block >= 0 && num_poi > 0)
            {
                char *ptr = line;
                int x, y;
                int i = 0; // Index for the coordinate pairs

                while (i < num_poi && (ptr = strchr(ptr, '{')) != NULL)
                {
                    if (sscanf(ptr, "{%d, %d}", &x, &y) == 2)
                    {
                        if (line_in_block < size_set_of_target && i < num_poi)
                        {
                            blocks[current_block][line_in_block][i][0] = x;
                            blocks[current_block][line_in_block][i][1] = y;
                            i++;
                        }
                    }

                    // Move the pointer forward after successfully parsing
                    ptr = strchr(ptr, '}');
                    if (ptr != NULL)
                    {
                        ptr++;
                    }
                }

                line_in_block++;
            }
        }
    }

    return blocks;
}

// Function to print a single block
void print_block(int ***block, int size_set_of_target, int num_poi)
{
    printf("{\n");
    for (int j = 0; j < size_set_of_target; j++) // Loop through each row
    {
        printf(" {");
        for (int k = 0; k < num_poi; k++) // Loop through each coordinate pair in the row
        {
            if (block[j] == NULL || block[j][k] == NULL)
            {
                printf(" NULL ");
                continue;
            }
            printf(" { %d, %d }", block[j][k][0], block[j][k][1]);
            if (k < num_poi - 1)
            {
                printf(", ");
            }
        }
        printf(" }");
        if (j < size_set_of_target - 1) // Assuming rows should be separated by commas
        {
            printf(",\n");
        }
        else
        {
            printf("\n");
        }
    }
    printf("}\n\n");
}

// Function to iterate over all blocks and print one by one
void print_all_blocks(int ****blocks, int *num_poi_array, int size_set_of_target, int block_count)
{
    for (int i = 0; i < block_count; i++)
    {
        printf("Block %d with num_poi: %d\n", i, num_poi_array[i]);

        if (blocks[i] == NULL)
        {
            printf("{\n { NULL },\n { NULL },\n { NULL }\n}\n\n");
            continue;
        }

        if (num_poi_array[i] == 0) // Handle empty num_poi case
        {
            printf("{\n { },\n { },\n { }\n}\n\n");
            continue;
        }

        print_block(blocks[i], size_set_of_target, num_poi_array[i]); // Print each block
    }
}

// Function to get a specific block by index
int ***get_block(int ****blocks, int block_index)
{
    return blocks[block_index];
}

int **get_set_targets(int ***block, int set_index)
{
    return block[set_index];
}

// Function to print a specific set (row) of coordinates
void print_set(int **set, int num_poi)
{
    printf("{ ");
    for (int i = 0; i < num_poi; i++)
    {
        printf("{ %d, %d }", set[i][0], set[i][1]);
        if (i < num_poi - 1)
        {
            printf(", ");
        }
    }
    printf(" }\n");
}

// Example function to demonstrate getting and printing specific blocks
void print_specific_block(int ****blocks, int *num_poi_array, int size_set_of_target, int block_index)
{
    int ***block = get_block(blocks, block_index); // Get the specific block
    printf("num_poi: %d\n", num_poi_array[block_index]);
    print_block(block, size_set_of_target, num_poi_array[block_index]); // Print the block
}

int find_number_of_found_targets(Target *targets, int targets_size)
{
    int number_targets_found = 0;
    for (int i = 0; i < targets_size; i++)
    {
        if (targets[i].found == 1)
            number_targets_found++;
    }
    return number_targets_found;
}

void reset_target_found(Target *targets, int targets_size)
{
    for (int i = 0; i < targets_size; i++)
    {
        targets[i].found = false;
    }
}

int max_run = 1;
// int min_drones = 20;
// int max_drones = 230;
int main(int argc, char *argv[])
{
    FILE *benchmark;
    benchmark = fopen("../benchmark/benchmark_random_targets_VESPA.csv", "w");
    fprintf(benchmark, "num_targets\tMin_num_drones\tMean_num_drones\tStd_num_drones\tMax_num_drones\tMin_num_drones_irr\tMean_num_drones_irr\tStd_num_drones_irr\tMax_num_drones_irr\n");
    //    srand(0);
    FILE *fp;
    fp = fopen("output.txt", "w");
    FILE *file_targaets = fopen("../benchmark/SAS/random_targets.txt", "r");
    int block_count;
    int size_set_of_target = 200;
    int *num_poi_array = malloc(30 * sizeof(int));
    int ****blocks = read_coordinates(file_targaets, num_poi_array, size_set_of_target, &block_count);
    // print_all_blocks(blocks, num_poi_array, size_set_of_target, block_count);
    for (int num_targets = 1; num_targets <= block_count; num_targets++)
    {

        int ***block = get_block(blocks, num_targets - 1);
        int min_numdrones = INT_MAX;
        int max_numdrones = 0;
        double sum_squares_numdrones = 0.0;
        float average_numdrones = 0;
        int sum_squared_numdrones = 0;
        int sum_squared_numdrones_used_drones_irr = 0;
        int num_fond = 0;
        float counter = 0;
        int num_used_drones = 0;
        // number_of_used_drones_irr
        float average_number_of_used_drones_irr = 0;
        int min_drones_used_irr = INT_MAX;
        int max_drones_used_irr = 0;

        for (int j = 0; j < size_set_of_target; j++)
        {
            int **set_ = get_set_targets(block, j);
            Target targets[num_targets];

            float predefinedTargets[num_targets][2];
            for (int i = 0; i < num_targets; i++)
            {
                predefinedTargets[i][0] = (float)set_[i][0];
                predefinedTargets[i][1] = (float)set_[i][1];
            }
            generate_targets(targets, num_targets, predefinedTargets);
            save_targes(targets, num_targets, fp);

            for (int numdrones_i = 4; numdrones_i <= 250; numdrones_i = numdrones_i + 1)
            {
                int numdrones = numdrones_i;
                bool certain_all_run_find = false;
                int counter_run_all_found = 0;
                // for (int run = 0; run < max_run; run++)
                // {
                srand((unsigned int)time(NULL));          // seed should be called one time not in the function that will be called each time
                reset_target_found(targets, num_targets); // need to make the targt unfound for the next iteration
                bool all_found = false;
                int targets_found_inround = 0;

                Drones drones[numdrones];
                initializeDrones(drones, numdrones);
                struct Neighbors DroneNeighbors[numdrones];
                for (int i = 0; i < numdrones; i++)
                {
                    initNeighbors(&DroneNeighbors[i]);
                }

                for (int i = 0; i < numdrones; i++)
                {
                    int dir = randomInt(1, 6);
                    moveDrones(&drones[i], dir);
                }
                saveDrones(drones, numdrones, fp);

                for (int i = 0; i < numdrones; i++)
                {
                    creatSpots(&DroneNeighbors[i], drones[i].x, drones[i].y);
                }

                perform_first_expansion(drones, DroneNeighbors, numdrones, fp);
                form_border_and_update_states(drones, DroneNeighbors, numdrones, targets, num_targets, fp);
                perform_spanning(drones, DroneNeighbors, numdrones, fp);
                perform_balancing_phase(drones, DroneNeighbors, numdrones, fp);

                targets_found_inround = find_number_of_found_targets(targets, num_targets);
                if (targets_found_inround == num_targets)
                {
                    all_found = true;
                    numdrones_i = 255;
                }

                for (int i = 0; i < numdrones; i++)
                {
                    drones[i].previous_state = drones[i].state;
                }

                int num_drones_border_irrmovable = 0;
                for (int i = 0; i < numdrones; i++)
                {
                    if (drones[i].state == Border || drones[i].state == Irremovable || drones[i].state == Irremovable_border)
                        num_drones_border_irrmovable++;
                }

                while (num_drones_border_irrmovable < numdrones)
                {
                    targets_found_inround = 0;
                    num_drones_border_irrmovable = 0;
                    perform_further_expansion(drones, DroneNeighbors, numdrones, fp);
                    form_further_border_and_update_states(drones, DroneNeighbors, numdrones, targets, num_targets, fp);
                    perform_further_spanning(drones, DroneNeighbors, numdrones, fp);
                    perform_balancing_phase(drones, DroneNeighbors, numdrones, fp);

                    for (int i = 0; i < numdrones; i++)
                    {
                        if (drones[i].state == Border || drones[i].state == Irremovable || drones[i].state == Irremovable_border)
                            num_drones_border_irrmovable++;
                    }
                    for (int i = 0; i < numdrones; i++)
                    {
                        if (drones[i].previous_state != Irremovable)
                            drones[i].previous_state = drones[i].state;
                    }

                    targets_found_inround = find_number_of_found_targets(targets, num_targets);

                    if (targets_found_inround == num_targets)
                    {
                        all_found = true;
                    }
                }
                int number_of_irr_drones = 0;
                for (int i = 0; i < numdrones; i++)
                {
                    if (drones[i].state == Irremovable || drones[i].state == Irremovable_border)
                        number_of_irr_drones++;
                }
                free_memory_drone(drones, numdrones);
                if (all_found)
                {
                    numdrones_i = 255;
                    num_used_drones = numdrones;
                    counter_run_all_found++;
                    if (numdrones < min_numdrones)
                    {
                        min_numdrones = numdrones;
                    }
                    if (numdrones > max_numdrones && counter_run_all_found == max_run)
                    {
                        max_numdrones = numdrones;
                    }
                    counter++;
                    average_numdrones = average_numdrones + num_used_drones;
                    sum_squared_numdrones += num_used_drones * num_used_drones;

                    if (number_of_irr_drones < min_drones_used_irr)
                    {
                        min_drones_used_irr = number_of_irr_drones;
                    }
                    if (number_of_irr_drones >= max_drones_used_irr)
                    {
                        max_drones_used_irr = number_of_irr_drones;
                    }
                    average_number_of_used_drones_irr += number_of_irr_drones;
                    sum_squared_numdrones_used_drones_irr += number_of_irr_drones * number_of_irr_drones;
                }
                //}
            }
        }
        average_numdrones = average_numdrones / counter;
        float variance_numdrones = (sum_squared_numdrones / (float)size_set_of_target) - (average_numdrones * average_numdrones);
        float standard_deviation_numdrones = sqrt(variance_numdrones);

        average_number_of_used_drones_irr = average_number_of_used_drones_irr / counter;
        float variance_numdrones_used_drones_irr = (sum_squared_numdrones_used_drones_irr / (float)size_set_of_target) - (average_number_of_used_drones_irr * average_number_of_used_drones_irr);
        float standard_deviation_numdrones_used_drones_irr = sqrt(variance_numdrones_used_drones_irr);

        printf("\nnum of targets found= %d\n", num_targets);
        printf("Average found= %f\n", average_numdrones);
        printf("Minimum found= %d\n", min_numdrones);
        printf("Maximum found= %d\n", max_numdrones);
        printf("standard_deviation_numdrones= %f\n", standard_deviation_numdrones);
        printf("min_drones_used_irr = %d\n", min_drones_used_irr);
        printf("max_drones_used_irr= %d\n", max_drones_used_irr);
        printf("average_number_of_used_drones_irr= %f\n", average_number_of_used_drones_irr);
        printf("standard_deviation_numdrones_used_drones_irr= %f\n", standard_deviation_numdrones_used_drones_irr);

        fprintf(benchmark, "%d\t%d\t%f\t%f\t%d\t%d\t%f\t%f\t%d\n",
                num_targets,
                min_numdrones,
                average_numdrones,
                standard_deviation_numdrones,
                max_numdrones,
                min_drones_used_irr,
                average_number_of_used_drones_irr,
                standard_deviation_numdrones_used_drones_irr,
                max_drones_used_irr);
    }
    // Free memory
    for (int i = 0; i < block_count; i++)
    {
        if (blocks[i] != NULL)
        {
            for (int j = 0; j < size_set_of_target; j++)
            {
                for (int k = 0; k < num_poi_array[i]; k++)
                {
                    free(blocks[i][j][k]);
                }
                free(blocks[i][j]);
            }
            free(blocks[i]);
        }
    }
    free(blocks);
    free(num_poi_array);

    fclose(fp);
    fclose(file_targaets);
    return 0;
}
