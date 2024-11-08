import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

output_directory = "../Fig_results/"
# Create the directory if it does not exist
os.makedirs(output_directory, exist_ok=True)

Sas_path= './SAS/num_drones_targets.csv'
Sas = pd.read_csv(Sas_path, sep='\t',engine='python')
VESPA_path= 'benchmark_random_targets_VESPA.csv'
VESPA_data = pd.read_csv(VESPA_path, sep='\t',engine='python')


VESPA_data['Upper'] = VESPA_data['Mean_num_drones'] + VESPA_data['Std_num_drones']
VESPA_data['Lower'] = VESPA_data['Mean_num_drones'] - VESPA_data['Std_num_drones']


Sas['Upper'] = Sas['SAS-mean'] + Sas['SAS-std']
Sas['Lower'] = Sas['SAS-mean'] - Sas['SAS-std']


plt.figure(figsize=(10, 6))
# Plot VESPA data
plt.plot(VESPA_data["num_targets"].values, VESPA_data["Mean_num_drones"].values, color='red', label='VESPA Mean', linewidth=0.8, marker='o',markersize=2)
plt.fill_between(VESPA_data["num_targets"], VESPA_data['Lower'].values, VESPA_data['Upper'].values, color='orange', alpha=0.3, label='VESPA Variance')

# Plot SAS data
plt.plot(Sas["n"].values, Sas["SAS-mean"].values, color='black', label='SAS Mean', linewidth=0.8, marker='s',markersize=2)
plt.fill_between(Sas["n"], Sas['Lower'].values, Sas['Upper'].values, color='purple', alpha=0.3,label='SAS Variance')

plt.xlabel('Number of Targets', fontsize=14, labelpad=10)
plt.ylabel('Drones Needed to Locate \n All Targets', fontsize=14, labelpad=10)
plt.xticks(fontsize=12)
xticks = np.arange( VESPA_data['num_targets'].min(), Sas['n'].max()+1)
plt.xticks(xticks, fontsize=12)
plt.yticks(np.arange(0,200, 10), fontsize=12)
plt.legend(loc='lower right', fontsize=12)
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.tight_layout()
plt.savefig(f"{output_directory}Drones Needed to Locate All Targets.png", format='png', dpi=300)


fig, ax1 = plt.subplots(figsize=(12, 8))
ax1.plot(VESPA_data["num_targets"], VESPA_data["Min_num_drones"], color='#FF8C00', label='VESPA Min', linewidth=1.5, marker='o', markersize=5)
ax1.plot(VESPA_data["num_targets"], VESPA_data["Max_num_drones"], color='#FF8C00', linestyle=':', label='VESPA Max', linewidth=1.5, marker='o', markersize=5)
ax1.fill_between(VESPA_data["num_targets"], VESPA_data["Min_num_drones"], VESPA_data["Max_num_drones"], color='#FFD580', alpha=0.3)

ax1.set_xlabel('Number of Targets', fontsize=14, labelpad=10)
ax1.set_ylabel('Max/Min VESPA Number of Drones', fontsize=14, color='#FF8C00')
ax1.tick_params(axis='y', labelcolor='#FF8C00')
ax1.grid(True, which='both', linestyle='--', linewidth=0.5)

ax2 = ax1.twinx()
ax2.plot(Sas["n"], Sas["SAS-min"], color='#800080', label='SAS Min', linewidth=1.5, marker='s', markersize=5)
ax2.plot(Sas["n"], Sas["SAS-max"], color='#800080', linestyle=':', label='SAS Max', linewidth=1.5, marker='s', markersize=5)
ax2.fill_between(Sas["n"], Sas["SAS-min"], Sas["SAS-max"], color='#D580FF', alpha=0.3)

ax2.set_ylabel('Max/Min SAS Number of Drones', fontsize=14, color='#800080')
ax2.tick_params(axis='y', labelcolor='#800080')

fig.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=4, fontsize=12)
plt.tight_layout(rect=[0, 0.05, 1, 0.95])
plt.savefig(f"{output_directory}Max-Min SAS Number of Drones.png", format='png', dpi=300)


# Convert to DataFrame for calculation
df = pd.DataFrame({'SAS_mean':  Sas['SAS-mean'], 'VESPA_mean': VESPA_data['Mean_num_drones']})
# Calculate the percentage improvement of VESPA over SAS
df['VESPA_improvement_percentage'] = ((df['SAS_mean'] - df['VESPA_mean']) / df['SAS_mean']) * 100
average_improvement_percentage = df['VESPA_improvement_percentage'].mean()
# Display the result
print("Number of drone need VESPA is better with", average_improvement_percentage, "%")




VESPA_data['Upper_irr'] = VESPA_data['Mean_num_drones_irr'] + VESPA_data['Std_num_drones_irr']
VESPA_data['Lower_irr'] = VESPA_data['Mean_num_drones_irr'] - VESPA_data['Std_num_drones_irr']

Sas_path_num_irr= './SAS/num_irr_drones_targets.csv'
Sas_num_drone = pd.read_csv(Sas_path_num_irr, sep='\t',engine='python')
Sas_num_drone['Upper_irr'] = Sas_num_drone['SAS-mean'] + Sas_num_drone['SAS-std']
Sas_num_drone['Lower_irr'] = Sas_num_drone['SAS-mean'] - Sas_num_drone['SAS-std']


# Plotting NUMBER of drones irremovalbles
plt.figure(figsize=(10, 6))
    
plt.plot(VESPA_data["num_targets"].values, VESPA_data["Mean_num_drones_irr"].values, color='red', label='VESPA Mean', linewidth=0.8, marker='o',markersize=2)
plt.fill_between(VESPA_data["num_targets"], VESPA_data['Upper_irr'].values, VESPA_data['Lower_irr'].values, color='orange', alpha=0.3, label='VESPA Variance')

plt.plot(Sas_num_drone["n"].values, Sas_num_drone["SAS-mean"].values, color='black', label='SAS Mean', linewidth=0.8, marker='s',markersize=2)
plt.fill_between(Sas_num_drone["n"], Sas_num_drone['Lower_irr'].values, Sas_num_drone['Upper_irr'].values, color='purple', alpha=0.3,label='SAS Variance')

plt.xlabel('Number of Targets', fontsize=14, labelpad=10)
plt.ylabel('Required Drones Irremovable\n for Connectivity', fontsize=14, labelpad=10)
plt.xticks(fontsize=12)
xticks = np.arange( VESPA_data['num_targets'].min(), Sas_num_drone['n'].max()+1)
plt.xticks(xticks, fontsize=12)
plt.yticks(np.arange(0,80, 5), fontsize=12)
plt.legend(loc='lower right', fontsize=12)
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.tight_layout()
plt.savefig(f"{output_directory}Required Drones Irremovable for Connectivity.png", format='png', dpi=300)


# Convert to DataFrame for calculation
df2 = pd.DataFrame({'SAS_mean':   Sas_num_drone['SAS-mean'], 'VESPA_mean': VESPA_data['Mean_num_drones_irr']})
# Calculate the percentage improvement of VESPA over SAS
df2['VESPA_improvement_percentage'] = ((df2['SAS_mean'] - df2['VESPA_mean']) / df2['SAS_mean']) * 100
average_improvement_percentage2 = df2['VESPA_improvement_percentage'].mean()
# Display the result
print("Number of Irremovable drones VESPA is better with", average_improvement_percentage2, "%")



