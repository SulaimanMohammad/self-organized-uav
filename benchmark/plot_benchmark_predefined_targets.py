import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

output_directory = "../Fig_results/"
# Create the directory if it does not exist
os.makedirs(output_directory, exist_ok=True)

VESPA_path = 'benchmark_predefined_targets_VESPA.csv'
Sas_path= './SAS/benchmark_8_targets_sas.csv'
# Read the CSV file
VESPA_data = pd.read_csv(VESPA_path, sep='\t')
Sas = pd.read_csv(Sas_path, sep='\t')

VESPA_data['Upper'] = VESPA_data['Mean'] + VESPA_data['Std']
VESPA_data['Lower'] = VESPA_data['Mean'] - VESPA_data['Std']

Sas['Upper'] = Sas['SAS-mean'] + Sas['SAS-std']
Sas['Lower'] = Sas['SAS-mean'] - Sas['SAS-std']


plt.figure(figsize=(10, 6))
plt.plot(VESPA_data["n"].values, VESPA_data["Mean"].values, color='black', label='VESPA Mean', linewidth=0.8, marker='o',markersize=2)
plt.fill_between(VESPA_data["n"], VESPA_data['Lower'].values, VESPA_data['Upper'].values, color='orange', alpha=0.3, label='VESPA Variance')
plt.plot(Sas["n"].values, Sas["SAS-mean"].values, color='black', label='SAS Mean', linewidth=0.8, marker='s',markersize=2)
plt.fill_between(Sas["n"], Sas['Lower'].values, Sas['Upper'].values, color='purple', alpha=0.3, label='SAS Variance')
plt.xlabel('Number of Drones', fontsize=14, labelpad=10)
plt.ylabel('Average Number of Discovered\n Targets (Variance)', fontsize=14, labelpad=10)
plt.xticks(fontsize=12)
xticks = np.arange( VESPA_data['n'].min(), Sas['n'].max() + 1, 10)  #multiple of 5
plt.xticks(xticks, fontsize=12)
plt.yticks(np.arange(0,9, 1), fontsize=12)
plt.legend(loc='lower right', fontsize=12)
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.tight_layout()
plt.savefig(f"{output_directory}num drones VESPA vs SAS 8 Targets.png", format='png', dpi=300)


## Find the df_performance percentage
# Include 'n' values (number of drones) to track discovery as drone count increases
n_values = list(range(5, 205, 5))  # Generating n values from 5 to 200 in increments of 5

# Recreate DataFrame with 'n', SAS_mean, VESPA_mean, and percentage improvement
df_performance = pd.DataFrame({
    'n': VESPA_data["n"],
    'SAS_mean': Sas["SAS-mean"],
    'VESPA_mean': VESPA_data["Mean"]
})

# Calculate the percentage improvement of VESPA over SAS in target discovery for each 'n'
df_performance['VESPA_improvement_percentage'] = (
    ((df_performance['VESPA_mean'] - df_performance['SAS_mean']) / df_performance['SAS_mean'].replace(0, float('nan'))) * 100
).fillna(0)

# Calculate the average percentage improvement across all values of 'n'
average_improvement_percentage_across_n = df_performance['VESPA_improvement_percentage'].mean()
# Display the average improvement
print("VESPA demonstrates a", f"{average_improvement_percentage_across_n:.2f}%", "improvement over all in compare with SaS")

df_filtered = df_performance[(df_performance['SAS_mean'] >= 7.95) | (df_performance['VESPA_mean'] >= 7.95)]

# Calculate the average percentage improvement across the filtered values of 'n'
average_improvement_percentage_filtered = df_filtered['VESPA_improvement_percentage'].mean()

# Display the average improvement for the filtered condition
print("VESPA demonstrates a",f"{average_improvement_percentage_filtered:.2f}%", " of improvement to find all targets over SAS.")

plt.figure(figsize=(10, 6))
positions = VESPA_data['n'].values
for i, n in enumerate(positions):
    vespa_subset = VESPA_data[VESPA_data['n'] == n]

    vespa_boxplot_data = np.concatenate([vespa_subset['Min'].values,
                                         vespa_subset['Mean'].values,
                                         vespa_subset['Max'].values])

    plt.boxplot(vespa_boxplot_data, positions=[n], widths=3, patch_artist=True,
                boxprops=dict(facecolor="orange", color="orange",alpha=0.5),
               medianprops=dict(color="black"),
               whiskerprops=dict(color="orange"),
              capprops=dict(color="orange"),
               showfliers=False)

    # Getting the current subset for Sas
    sas_subset = Sas[Sas['n'] == n]

    sas_boxplot_data = np.concatenate([sas_subset['SAS-min'].values,
                                       sas_subset['SAS-mean'].values,
                                       sas_subset['SAS-max'].values])

    plt.boxplot(sas_boxplot_data, positions=[n], widths=3, patch_artist=True,
                boxprops=dict(facecolor="purple", color="purple", alpha=0.5),
                 medianprops=dict(color="black"),
               whiskerprops=dict(color="purple"),
              capprops=dict(color="purple"),
               showfliers=False)

plt.xlabel('Number of Drones', fontsize=12)
plt.ylabel('Max/Avg/Max Number \n of Discovered Targets', fontsize=12)
plt.yticks(np.arange(0,9, 1), fontsize=12)
xticks_range = np.arange(start=VESPA_data['n'].min(), stop=VESPA_data['n'].max() + 1, step=10)
plt.xticks(ticks=xticks_range, labels=xticks_range)
plt.xlim(left=0)
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
legend_elements = [Patch(facecolor='orange', edgecolor='orange', label='VESPA', alpha=0.5),
                   Patch(facecolor='purple', edgecolor='purple', label='SAS', alpha=0.5)]
plt.legend(handles=legend_elements, loc='lower right')
plt.savefig(f"{output_directory}Max-Avg-Max num drones VESPA vs SAS 8 Targets.png", format='png', dpi=300)



# Filter out rows in VESPA_data where any of the specified columns are -1 and make a copy
filtered_VESPA_data = VESPA_data[(VESPA_data['Mean_round'] != -1) &
                                 (VESPA_data['Std_round'] != -1) &
                                 (VESPA_data['Var_round'] != -1) &
                                 (VESPA_data['Max_round'] != -1)].copy()

# Now, set the new columns without triggering the warning
filtered_VESPA_data['Upper_filt'] = filtered_VESPA_data['Mean_round'] + filtered_VESPA_data['Std_round']
filtered_VESPA_data['Lower_filt'] = filtered_VESPA_data['Mean_round'] - filtered_VESPA_data['Std_round']
plt.figure(figsize=(10, 6))
# Plot VESPA data
plt.plot(filtered_VESPA_data["n"].values, filtered_VESPA_data["Mean_round"].values, color='black', label='VESPA Mean', linewidth=0.8, marker='o',markersize=2)
plt.fill_between(filtered_VESPA_data["n"], filtered_VESPA_data['Lower_filt'].values, filtered_VESPA_data['Upper_filt'].values, color='orange', alpha=0.3, label='VESPA Variance')
plt.xlabel('Number of Drones', fontsize=14, labelpad=10)
plt.ylabel('Number and Variance\n of 3-phase Round', fontsize=14, labelpad=10)
plt.xticks(np.arange(min(filtered_VESPA_data["n"]), max(filtered_VESPA_data["n"]) + 1, 10), fontsize=12)
plt.xticks(fontsize=12)
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.tight_layout()
plt.savefig(f"{output_directory}Number and Variance 3-phase Round.png", format='png', dpi=300)



plt.figure(figsize=(10, 6))
# Define the positions for the boxplots, which will be the unique values in 'n'
positions = filtered_VESPA_data['n'].unique()
positions.sort()  # Ensure positions are in ascending order
# Plot boxplot for each position
for n in positions:
    # Select the subset of data for the current 'n'
    subset = filtered_VESPA_data[filtered_VESPA_data['n'] == n]

    # Combine the metrics into a single array for the boxplot
    combined_metrics = np.concatenate([
        subset['Mean_round'].values,
        subset['Std_round'].values,
        subset['Var_round'].values,
        subset['Max_round'].values
    ])

    # Create a boxplot or a scatter point depending on the variation in the data
    if np.min(combined_metrics) == np.max(combined_metrics):
        # If there is no variation, plot a scatter point instead of a boxplot
        plt.scatter([n], [np.min(combined_metrics)], color='red', zorder=3)
    else:
        # Plot the boxplot at the position 'n'
        plt.boxplot(combined_metrics, positions=[n], widths=2, patch_artist=True,
                    boxprops=dict(facecolor="orange", color="orange", alpha=0.5),
                    medianprops=dict(color="black"),
                    whiskerprops=dict(color="orange"),
                    capprops=dict(color="orange"),
                    showfliers=False)
plt.xlabel('Number of Drones', fontsize=12)
plt.ylabel('Max/Avg/Max Number \n of 3-phase Round', fontsize=12)
plt.xticks(np.arange(min(positions), max(positions) + 1, 5))
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.tight_layout()
plt.savefig(f"{output_directory}Max-Avg-Max Number and Variance 3-phase Round.png", format='png', dpi=300)
