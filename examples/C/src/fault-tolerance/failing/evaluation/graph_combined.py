import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

# Load and clean CSV
df = pd.read_csv("res100k/res100k.csv")
df.columns = df.columns.str.strip()

# Organize data by strategy
wcet_opt_data = df.iloc[0:4].reset_index(drop=True)
baseline_data = df.iloc[4:8].reset_index(drop=True)
wcet_data = df.iloc[8:12].reset_index(drop=True)

# Reorder bars for grouped visualization
combined_df = pd.DataFrame([
    baseline_data.iloc[0], wcet_data.iloc[0], wcet_opt_data.iloc[0],
    baseline_data.iloc[1], wcet_data.iloc[1], wcet_opt_data.iloc[1],
    baseline_data.iloc[2], wcet_data.iloc[2], wcet_opt_data.iloc[2],
    baseline_data.iloc[3], wcet_data.iloc[3], wcet_opt_data.iloc[3],
]).reset_index(drop=True)

# Common settings
strategy_labels = ['Baseline', 'WCET', 'OPT'] * 4
failure_rate_labels = ['0.5%', '1%', '5%', '10%']
x = np.array([0, 1, 2, 3.2, 4.2, 5.2, 6.4, 7.4, 8.4, 9.6, 10.6, 11.6])
bar_width = 0.95
plt.rcParams['font.family'] = 'Times New Roman'

# Legend labels, colors, and hatches
legend_labels = [
    'Baseline', 'Proposed1', 'Proposed2'
]
legend_colors = [
    '#1f77b4', '#2ca02c', '#9467bd',
    '#1f77b4', '#2ca02c', '#9467bd',
    '#1f77b4', '#2ca02c', '#9467bd',
    '#1f77b4', '#2ca02c', '#9467bd'
]

bar_colors = legend_colors

def plot_single_metric(ax, y_data, title, ylabel, value_format='int'):
    for i in range(len(x)):
        ax.bar(x[i], y_data[i], width=bar_width, color=bar_colors[i], alpha=0.85,
                edgecolor='black')
        val = f'{int(y_data[i])}' if value_format == 'int' else f'{y_data[i]:.2f}'
        ax.text(x[i], y_data[i] + 1, val, ha='center', va='bottom', fontsize=16, rotation=(90))

    # Remove default x-ticks
    ax.set_xticks([])

    # Group labels (bottom row)
    group_positions = [np.mean(x[i:i + 3]) for i in range(0, len(x), 3)]
    for pos, label in zip(group_positions, failure_rate_labels):
        ax.text(pos, -0.05 * max(y_data), label, ha='center', va='top', fontsize=18)

    ax.set_xlim(-0.7, 12)
    ax.set_ylim(bottom=0, top=max(y_data) + (20 if value_format == 'float' else 220))
    # ax.set_ylabel(ylabel, fontsize=21, labelpad=-3)
    ax.tick_params(axis='y', labelsize=17)
    ax.grid(axis='y', linestyle='--', alpha=0.5)
    # ax.set_title(title, fontsize=18)

# Create a single PDF with all plots side by side
with PdfPages("res420/graph.pdf") as pdf, \
     PdfPages("/Users/dkim314/project/TCRS2025/graph.pdf") as pdf2:
    fig, axs = plt.subplots(1, 4, figsize=(20, 4))
    total_failure = combined_df['Deadline_miss'] + combined_df['Execution_failed']
    plot_single_metric(axs[0], total_failure, 'Total Failures', 'Total Task Failure Count')
    plot_single_metric(axs[1], combined_df['Execution_failed'], 'Execution Failures', 'Execution Fail Count')
    plot_single_metric(axs[2], combined_df['Deadline_miss'], 'Deadline Misses', 'Deadline Miss Count')
    plot_single_metric(axs[3], combined_df['Utilization'], 'CPU Utilization', 'CPU Utilization (%)', value_format='float')

    # Add colored legend with hatching in one line above all plots
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=col, label=lab,  edgecolor='black')
                       for col, lab in zip(legend_colors, legend_labels)]
    fig.legend(legend_elements, legend_labels, loc='upper center', ncol=len(legend_labels), fontsize=18,
               frameon=False, bbox_to_anchor=(0.5, 0.98), handletextpad=0.5, columnspacing=0.8)

    # plt.tight_layout(rect=[0, 0, 1, 0.88])  # Adjust space to fit legend
    fig.subplots_adjust(wspace=0.15, top=0.85, bottom=0.2)
    # Add subfigure labels below each subplot
    subfigure_labels = [
        "(a) Total task failures.",
        "(b) Execution failures.",
        "(c) Deadline misses.",
        "(d) CPU utilization (%)."
    ]

    # Position each label centered below the corresponding subplot
    for ax, label in zip(axs, subfigure_labels):
        ax_pos = ax.get_position()
        fig.text(x=ax_pos.x0 + ax_pos.width / 2,
                 y=ax_pos.y0 - 0.08,
                 s=label,
                 ha='center',
                 va='top',
                 fontsize=24)
    fig.text(0.11, 0.172, 'Fault Rate', ha='center', va='top', fontsize=16)
    pdf.savefig(fig, bbox_inches='tight')
    pdf2.savefig(fig, bbox_inches='tight')
    plt.close()