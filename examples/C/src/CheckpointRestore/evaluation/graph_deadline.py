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

# Plot settings
bar_width = 0.95
colors = ['#1f77b4', '#2ca02c']
hatches = ['\\', '//']

plt.rcParams['font.family'] = 'Times New Roman'
fig, ax = plt.subplots(figsize=(5, 3.5))

# Plot stacked bars
ax.bar(x, combined_df['Deadline_miss'], width=bar_width,
       color=colors[0], label='Deadline\nMiss',
       hatch=hatches[0], alpha=0.95)

ax.bar(x, combined_df['Execution_failed'], width=bar_width,
       bottom=combined_df['Deadline_miss'], color=colors[1],
       label='Execution\nFail', hatch=hatches[1])

# Remove default ticks
ax.set_xticks([])

# Draw strategy labels (top row)
for xi, label in zip(x, strategy_labels):
    ax.text(xi, -0, label, ha='center', va='top', fontsize=18, rotation=45)

# Draw group labels (bottom row)
group_positions = [np.mean(x[i:i+3]) for i in range(0, len(x), 3)]
for pos, label in zip(group_positions, failure_rate_labels):
    # ax.text(pos, -230, label, ha='center', va='top', fontsize=18, fontweight='bold')
    ax.text(pos, -300, label, ha='center', va='top', fontsize=18, fontweight='bold', rotation=45)

# # Label for the minor x-axis (strategy)
# # ax.text(-1.3, -10, "Strategy", ha='center', va='top', fontsize=18, rotation=45)
# ax.text(-1.5, -60, "Strategy", ha='center', va='top', fontsize=18)

# # Label for the major x-axis (fault rate group)
# # ax.text(-1.5, -130, "Fault Rate", ha='center', va='top', fontsize=18, rotation=45)
# ax.text(-1.7, -180, "Fault Rate", ha='center', va='top', fontsize=18)

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
    ax.text(x[i], total + 1, f'{int(total)}', ha='center', va='bottom', fontsize=18)

# Axis and legend
ax.set_xlim(-0.5, 9)
ax.set_ylabel('Task Failure Count', fontsize=18, labelpad=-5)
ax.tick_params(axis='y', labelsize=17)
ax.grid(axis='y', linestyle='--', alpha=0.5)

legend = ax.legend(
    fontsize=18,
    labelspacing=0.1,
    handletextpad=0.1,
    loc='upper left',
    bbox_to_anchor=(-0.05, 1.0)  # <-- Pushes it to the right
)
legend.get_frame().set_linewidth(0.0)
legend.get_frame().set_facecolor('none')

ax.set_ylim(top=combined_df['Deadline_miss'].add(combined_df['Execution_failed']).max() + 200)

# Save figure as PDF
plt.savefig("examples/C/src/CheckpointRestore/evaluation/res420/res420_deadline.pdf", format='pdf', bbox_inches='tight')

# plt.tight_layout()
plt.show()
