import pandas as pd
import matplotlib.pyplot as plt

# Set font and style
plt.rcParams['font.family'] = 'Times New Roman'

# Load data
airspeed_noretry = pd.read_csv("../airspeed.data", delim_whitespace=True, header=None, names=["time", "airspeed"])
altitude_noretry = pd.read_csv("../altitude.data", delim_whitespace=True, header=None, names=["time", "altitude"])
airspeed_retry = pd.read_csv("../airspeedRe-execution.data", delim_whitespace=True, header=None, names=["time", "airspeed"])
altitude_retry = pd.read_csv("../altitudeRe-execution.data", delim_whitespace=True, header=None, names=["time", "altitude"])

# Filter to time >= 40s
airspeed_noretry = airspeed_noretry[airspeed_noretry['time'] >= 40]
altitude_noretry = altitude_noretry[altitude_noretry['time'] >= 40]
airspeed_retry = airspeed_retry[airspeed_retry['time'] >= 40]
altitude_retry = altitude_retry[altitude_retry['time'] >= 40]

# Create subplots
fig, axs = plt.subplots(1, 2, figsize=(8, 3))
plt.subplots_adjust(wspace=0.001)
# plt.tight_layout()

# --- Left plot: No Retry ---
ax1 = axs[0]
ax1.set_xlabel("Time (s)", fontsize=11, color='black')
ax1.xaxis.set_label_coords(1.15, -0.045)  # x=1.0 puts it at far right near ticks
ax1.set_ylabel("Altitude (m)", fontsize=19, color='tab:blue')
ax1.plot(altitude_noretry['time'], altitude_noretry['altitude'], color='tab:blue')
ax1.tick_params(axis='both', labelsize=13, colors='black')
ax1.ticklabel_format(style='plain', axis='y', useOffset=False)

ax2 = ax1.twinx()
ax2.set_ylabel("Airspeed (m/s)", fontsize=19, color='tab:red')
ax2.plot(airspeed_noretry['time'], airspeed_noretry['airspeed'], color='tab:red')
ax2.tick_params(axis='y', labelsize=13, colors='black')
ax2.ticklabel_format(style='plain', axis='y', useOffset=False)

# Subfigure label
ax1.text(0.5, -0.55, "(a) Basline ROSACE software \nwithout fault tolerance.", fontsize=18, ha='center', transform=ax1.transAxes)

# --- Right plot: Retry ---
ax3 = axs[1]
ax3.set_xlabel("Time (s)", fontsize=13, color='black')
ax3.xaxis.set_label_coords(1.08, -0.045)  # x=1.0 puts it at far right near ticks
ax3.set_ylabel("Altitude (m)", fontsize=19, color='tab:blue')
ax3.plot(altitude_retry['time'], altitude_retry['altitude'], color='tab:blue')
ax3.tick_params(axis='both', labelsize=13, colors='black')
ax3.ticklabel_format(style='plain', axis='y', useOffset=False)

ax4 = ax3.twinx()
ax4.set_ylabel("Airspeed (m/s)", fontsize=19, color='tab:red')
ax4.plot(airspeed_retry['time'], airspeed_retry['airspeed'], color='tab:red')
ax4.tick_params(axis='y', labelsize=13, colors='black')
ax4.ticklabel_format(style='plain', axis='y', useOffset=False)

# Subfigure label
ax3.text(0.5, -0.55, "(b) ROSACE software with \nre-execution-based fault tolerance.", fontsize=18, ha='center', transform=ax3.transAxes)

# Remove top/right spines and add axis arrows
for ax in [ax1, ax2, ax3, ax4]:
    ax.spines['top'].set_visible(False)
    # ax.spines['right'].set_visible(False)

    # Add arrow to x-axis
    ax.annotate('', xy=(1.02, 0), xycoords='axes fraction',
                xytext=(0.98, 0), textcoords='axes fraction',
                arrowprops=dict(facecolor='black', arrowstyle='->', lw=1))

    # Add arrow to y-axis
    ax.annotate('', xy=(0, 1.02), xycoords='axes fraction',
                xytext=(0, 0.98), textcoords='axes fraction',
                arrowprops=dict(facecolor='black', arrowstyle='->', lw=1))
    
    # Add arrow to y-axis
    ax.annotate('', xy=(1, 1.02), xycoords='axes fraction',
                xytext=(1, 0), textcoords='axes fraction',
                arrowprops=dict(facecolor='black', arrowstyle='->', lw=1))

# Save figure
plt.tight_layout()
plt.savefig("graph.pdf", bbox_inches='tight')
