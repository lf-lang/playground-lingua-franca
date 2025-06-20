import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
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

def plot_single_metric(ax, y_data, value_format='int'):
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
    ax.set_ylim(bottom=0, top=(max(y_data) * 1.25))
    # ax.set_ylabel(ylabel, fontsize=21, labelpad=-3)
    ax.tick_params(axis='y', labelsize=17)
    ax.grid(axis='y', linestyle='--', alpha=0.5)
    # ax.set_title(title, fontsize=18)

def plot_single_metric_split3(ax, y_data):
    break_high = 5000
    break_mid_high = 2100
    break_mid_low = 2000
    break_low = 1020
    gs = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=ax, height_ratios=[5, 2, 10], hspace=0.2)
    ax_upper = fig.add_subplot(gs[0])
    ax_mid = fig.add_subplot(gs[1])
    ax_lower = fig.add_subplot(gs[2], sharex=ax_upper)

    for i in range(len(x)):
        ax_upper.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
        ax_mid.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
        ax_lower.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
        val = f'{int(y_data[i])}'
        if y_data[i] > break_mid_high:
            ax_upper.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
            ax_upper.text(x[i], y_data[i] + 1, val, ha='center', va='bottom', fontsize=16, rotation=90)
        elif y_data[i] > break_low:
            ax_mid.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
            ax_mid.text(x[i], y_data[i] + 1, val, ha='center', va='bottom', fontsize=16, rotation=90)
        else:
            ax_lower.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
            ax_lower.text(x[i], y_data[i] + 1, val, ha='center', va='bottom', fontsize=16, rotation=90)

    ax_upper.set_xticks([])
    ax_mid.set_xticks([])
    ax_lower.set_xticks([])

    # Group labels
    group_positions = [np.mean(x[i:i + 3]) for i in range(0, len(x), 3)]
    for pos, label in zip(group_positions, failure_rate_labels):
        ax_lower.text(pos, -0.05 * break_low, label, ha='center', va='top', fontsize=18)
    ax_upper.set_ylim(break_high, max(y_data) + 450)
    ax_mid.set_ylim(break_mid_low, break_mid_high)
    ax_lower.set_ylim(0, break_low)
    ax_upper.spines['bottom'].set_visible(False)
    ax_mid.spines['top'].set_visible(False)
    ax_mid.spines['bottom'].set_visible(False)
    ax_lower.spines['top'].set_visible(False)
    ax_upper.tick_params(labeltop=False)
    ax_lower.tick_params(axis='y', labelsize=15)
    ax_mid.tick_params(axis='y', labelsize=15)
    ax_upper.tick_params(axis='y', labelsize=15)

    # Break indicators
    d = .5
    kwargs = dict(marker=[(-1, -d), (1, d)], markersize=10,
                  linestyle="none", color='k', mec='k', mew=1, clip_on=False)
    # Between ax_upper and ax_mid
    ax_upper.plot([0, 1], [0, 0], transform=ax_upper.transAxes, **kwargs)
    ax_mid.plot([0, 1], [1, 1], transform=ax_mid.transAxes, **kwargs)

    # Between ax_mid and ax_lower
    ax_mid.plot([0, 1], [0, 0], transform=ax_mid.transAxes, **kwargs)
    ax_lower.plot([0, 1], [1, 1], transform=ax_lower.transAxes, **kwargs)
    return ax_lower

def plot_single_metric_split2(ax, y_data):
    break_high = 58
    break_low = 10
    gs = gridspec.GridSpecFromSubplotSpec(2, 1, subplot_spec=ax, height_ratios=[9, 1], hspace=0.2)
    ax_upper = fig.add_subplot(gs[0])
    ax_lower = fig.add_subplot(gs[1], sharex=ax_upper)
    for i in range(len(x)):
        val = f'{y_data[i]:.2f}'
        ax_upper.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
        ax_lower.bar(x[i], y_data[i], width=bar_width, color=legend_colors[i], edgecolor='black', alpha=0.85)
        ax_upper.text(x[i], y_data[i]+0.1, val, ha='center', va='bottom', fontsize=16, rotation=(90))

    # Remove xticks
    ax_upper.set_xticks([])
    ax_lower.set_xticks([])

    # Group labels
    group_positions = [np.mean(x[i:i + 3]) for i in range(0, len(x), 3)]
    for pos, label in zip(group_positions, failure_rate_labels):
        ax_lower.text(pos, -0.05 * break_low, label, ha='center', va='top', fontsize=18)
    ax_upper.set_ylim(break_high, max(y_data) + 5)
    ax_lower.set_ylim(0, break_low)
    ax_upper.spines['bottom'].set_visible(False)
    ax_lower.spines['top'].set_visible(False)
    ax_upper.tick_params(labeltop=False)
    ax_lower.tick_params(axis='y', labelsize=15)
    ax_upper.tick_params(axis='y', labelsize=15)

    # Break indicators
    d = .5
    kwargs = dict(marker=[(-1, -d), (1, d)], markersize=10,
                  linestyle="none", color='k', mec='k', mew=1, clip_on=False)
    ax_upper.plot([0, 1], [0, 0], transform=ax_upper.transAxes, **kwargs)
    ax_lower.plot([0, 1], [1, 1], transform=ax_lower.transAxes, **kwargs)

    return ax_lower

# Create a single PDF with all plots side by side
with PdfPages("res100k/graph.pdf") as pdf:
    # fig, axs = plt.subplots(1, 4, figsize=(20, 4))
    fig = plt.figure(figsize=(20, 4))
    outer = gridspec.GridSpec(1, 4, wspace=0.2)

    total_failure = combined_df['Deadline_miss'] + combined_df['Execution_failed']
    # ax0 = fig.add_subplot(outer[0])
    ax0 = fig.add_subplot(outer[2])
    plot_single_metric(ax0, total_failure)
    ax1 = fig.add_subplot(outer[1])
    plot_single_metric(ax1, combined_df['Execution_failed'])
    ax2 = fig.add_subplot(outer[0])
    plot_single_metric(ax2, combined_df['Deadline_miss'])
    # ax3 = fig.add_subplot(outer[3])
    ax3 = plot_single_metric_split2(outer[3], combined_df['Utilization'])

    axs= [ax2, ax1, ax0, ax3]

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
        "(a) Deadline misses.",
        "(b) Execution failures.",
        "(c) Total task failures.",
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
    fig.text(0.05, 0.18, 'Failure Rate', ha='center', va='top', fontsize=16)
    pdf.savefig(fig, bbox_inches='tight')
    plt.close()