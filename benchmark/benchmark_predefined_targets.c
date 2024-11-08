#include "expansion.h"
#include "spanning.h"
#include "balancing.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

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

int max_run = 100;
int min_drones = 5;
int max_drones = 200;
int main(int argc, char *argv[])
{
    FILE *benchmark;
    benchmark = fopen("../benchmark/benchmark_predefined_targets_VESPA.csv", "w");
    // fprintf(benchmark, "n\tMin\tMean\tStd\tVar\tMax\tMin_round\tMean_round\tStd_round\tVar_round\tMax_round\n");
    fprintf(benchmark, "n\tMin\tMean\tStd\tVar\tMax\tMin_round\tMean_round\tStd_round\tVar_round\tMax_round\tMin_used\tMean_used\tMax_used\n");
    const int targets_size = 8;

    FILE *fp;
    fp = fopen("output.txt", "w");
    Target targets[targets_size];

    for (int numdrones = min_drones; numdrones <= max_drones; numdrones = numdrones + 5)
    {

        // Coordinates of the specific targets
        // float predefinedTargets[8][2] = {
        //     {0, 100}, {0, -85 * 3}, {85 * 2, 0}, {-85 * 4, 60}, {60 * 2, 75}, {20 * 3, -75 * 2}, {-90, 300}, {-200, 150}};
        float predefinedTargets[8][2] = {
            {0, 85}, {0, -85}, {85, 0}, {-85, 0}, {75, 75}, {75, -75}, {-75, 75}, {-75, -75}};

        generate_targets(targets, targets_size, predefinedTargets);
        save_targes(targets, targets_size, fp);

        int min_targets_found = INT_MAX;
        int max_targets_found = 0;
        double sum_squares_targets_found = 0.0;
        int average_number_targets_found = 0;
        int average_number_of_used_drones = 0;
        int min_rounds = INT_MAX;
        int max_rounds = 0;

        int min_drones_used = INT_MAX;
        int max_drones_used = 0;

        double sum_squares_rounds = 0.0;
        int average_number_of_rounds = 0;
        float round_eligable = 0;
        srand((unsigned int)time(NULL)); // seed should be called one time not in the function that will be called each time

        for (int run = 0; run < max_run; run++)
        {

            reset_target_found(targets, targets_size); // need to make the targt unfound for the next iteration
            bool all_found = false;
            int previous_number_targets_found = 0;
            int number_of_used_drones = 0;
            int targets_found_inround = 0;
            int number_of_3_rounds = 0;

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
            form_border_and_update_states(drones, DroneNeighbors, numdrones, targets, targets_size, fp);
            perform_spanning(drones, DroneNeighbors, numdrones, fp);
            perform_balancing_phase(drones, DroneNeighbors, numdrones, fp);

            targets_found_inround = find_number_of_found_targets(targets, targets_size);
            if (targets_found_inround == targets_size)
            {
                all_found = true;
                number_of_3_rounds++;
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
                form_further_border_and_update_states(drones, DroneNeighbors, numdrones, targets, targets_size, fp);
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

                targets_found_inround = find_number_of_found_targets(targets, targets_size);

                if (targets_found_inround <= targets_size && all_found == false)
                {
                    number_of_3_rounds++;
                }

                if (targets_found_inround == targets_size)
                    all_found = true;
            }

            number_of_used_drones = 0;
            for (int i = 0; i < numdrones; i++)
            {
                if (drones[i].state != Irremovable && drones[i].state != Irremovable_border)
                    number_of_used_drones++;
            }

            saveDrones(drones, numdrones, fp);
            free_memory_drone(drones, numdrones);

            int number_targets_found = find_number_of_found_targets(targets, targets_size);

            if (number_targets_found < min_targets_found)
            {
                min_targets_found = number_targets_found;
            }
            if (number_targets_found > max_targets_found)
            {
                max_targets_found = number_targets_found;
            }

            if (all_found == true)
            {
                round_eligable++;
                if (number_of_3_rounds < min_rounds)
                {
                    min_rounds = number_of_3_rounds;
                }
                if (number_of_3_rounds > max_rounds)
                {
                    max_rounds = number_of_3_rounds;
                }

                average_number_of_rounds = average_number_of_rounds + number_of_3_rounds;
                sum_squares_rounds += number_of_3_rounds * number_of_3_rounds;
            }

            average_number_targets_found = average_number_targets_found + number_targets_found;
            sum_squares_targets_found += number_targets_found * number_targets_found;

            if (number_of_used_drones < min_drones_used)
            {
                min_drones_used = number_of_used_drones;
            }
            if (number_of_used_drones >= max_drones_used)
            {
                max_drones_used = number_of_used_drones;
            }
            average_number_of_used_drones = average_number_of_used_drones + number_of_used_drones;
        }

        double average_targets_found = average_number_targets_found / (float)max_run;
        double variance_targets_found = (sum_squares_targets_found / (float)max_run) - (average_targets_found * average_targets_found);
        double standard_deviation_targets_found = sqrt(variance_targets_found);
        double average_rounds;
        double variance_rounds;
        double standard_deviation_rounds;
        if (max_targets_found < targets_size)
        {
            average_rounds = -1;
            variance_rounds = -1;
            standard_deviation_rounds = -1;
            max_rounds = -1;
            min_rounds = -1;
        }
        else
        {
            average_rounds = average_number_of_rounds / round_eligable;
            variance_rounds = (sum_squares_rounds / round_eligable) - (average_rounds * average_rounds);
            standard_deviation_rounds = sqrt(variance_rounds);
        }
        double final_average_number_of_used_drones = average_number_of_used_drones / (float)max_run;
        fprintf(benchmark, "%d\t%d\t%f\t%f\t%f\t%d\t%d\t%f\t%f\t%f\t%d\t%d\t%f\t%d\n",
                numdrones,
                min_targets_found,
                average_targets_found,
                standard_deviation_targets_found,
                variance_targets_found,
                max_targets_found,
                min_rounds,
                average_rounds,
                standard_deviation_rounds,
                variance_rounds,
                max_rounds,
                min_drones_used,
                final_average_number_of_used_drones,
                max_drones_used);

        printf("\n number of drones= %d\n", numdrones);
        printf("\nAverage found= %f\n", average_targets_found);
        printf("Minimum found= %d\n", min_targets_found);
        printf("Maximum found= %d\n", max_targets_found);
        printf("Variance= %f\n", variance_targets_found);
        printf("Standard Deviation= %f\n", standard_deviation_targets_found);

        printf("Average_rounds= %f\n", average_rounds);
        printf("Minimum rounds= %d\n", min_rounds);
        printf("Maximum rounds= %d\n", max_rounds);
        printf("Variance rounds= %f\n", variance_rounds);
        printf("Standard Deviation rounds= %f\n", standard_deviation_rounds);

        printf("average_number_of_used_drones= %f\n", final_average_number_of_used_drones);
        printf("Minimum drones_used= %d\n", min_drones_used);
        printf("Maximum drones_used= %d\n", max_drones_used);
    }

    fclose(fp);
    fclose(benchmark);
    return 0;
}
