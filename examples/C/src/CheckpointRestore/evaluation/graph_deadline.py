import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load and clean CSV
# df = pd.read_csv("examples/C/src/CheckpointRestore/evaluation/res42/res42.csv")
df = pd.read_csv("examples/C/src/CheckpointRestore/evaluation/res420/res420.csv")
df.columns = df.columns.str.strip()

# Organize data by strategy
wcet_opt_data = df.iloc[0:3].reset_index(drop=True)
baseline_data = df.iloc[3:6].reset_index(drop=True)
wcet_data = df.iloc[6:9].reset_index(drop=True)

# Reorder bars
combined_df = pd.DataFrame([
    baseline_data.iloc[0], wcet_data.iloc[0], wcet_opt_data.iloc[0],
    baseline_data.iloc[1], wcet_data.iloc[1], wcet_opt_data.iloc[1],
    baseline_data.iloc[2], wcet_data.iloc[2], wcet_opt_data.iloc[2]
]).reset_index(drop=True)

# Labels
strategy_labels = ['Baseline', 'WCET', 'OPT'] * 3
failure_rate_labels = ['1%', '5%', '10%']
x = np.array([0, 1, 2, 3.2, 4.2, 5.2, 6.4, 7.4, 8.4])
# empty_x = [3, 7]

# Plot settings
bar_width = 0.95
colors = ['#1f77b4', '#2ca02c']
hatches = ['\\', '//']

plt.rcParams['font.family'] = 'Times New Roman'
fig, ax = plt.subplots(figsize=(5, 5))

# Plot stacked bars
bars1 = ax.bar(x, combined_df['Deadline_miss'],
               width=bar_width, color=colors[0],
               label='Deadline Miss', hatch=hatches[0], alpha=0.95)

bars2 = ax.bar(x, combined_df['Execution_failed'],
               width=bar_width,
               bottom=combined_df['Deadline_miss'],
               color=colors[1], hatch=hatches[1],
               label='Execution Fail')

# # Invisible spacer bars
# ax.bar(empty_x, [0]*len(empty_x), width=0, color='white')

# Remove default ticks
ax.set_xticks([])

# Draw strategy labels (top row)
for xi, label in zip(x, strategy_labels):
    ax.text(xi, -10, label, ha='center', va='top', fontsize=20, rotation=45)
    # ax.text(xi, -10, label, ha='center', va='top', fontsize=20)

# Draw group labels (bottom row)
group_positions = [np.mean(x[i:i+3]) for i in range(0, len(x), 3)]
for pos, label in zip(group_positions, failure_rate_labels):
    # ax.text(pos, -170, label, ha='center', va='top', fontsize=20, rotation=45, fontweight='bold')
    ax.text(pos, -230, label, ha='center', va='top', fontsize=20, fontweight='bold')

# Adjust bottom limit to fit both label rows
ax.set_ylim(bottom=0)

# # Draw divider lines
# for div in [2.6, 5.8]:
#     ax.plot([div, div], [-10, -310], color='black', linewidth=1.2, clip_on=False)

# Add total labels above each full bar
for i in range(len(x)):
    dm = combined_df['Deadline_miss'][i]
    ef = combined_df['Execution_failed'][i]
    total = dm + ef
    ax.text(x[i], total + 1, f'{int(total)}', ha='center', va='bottom', fontsize=15)

# Axis and legend
ax.set_xlim(-0.5, 9)
# ax.set_ylim(bottom=-15)
ax.set_ylabel('Task Failure Count', fontsize=20, labelpad=-10)
ax.tick_params(axis='y', labelsize=15)
ax.grid(axis='y', linestyle='--', alpha=0.5)
ax.legend(fontsize=15)

plt.tight_layout()
plt.show()