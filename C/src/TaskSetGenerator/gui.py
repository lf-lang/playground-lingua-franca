# A GUI program for task set generator.
# It helps to configure task sets to be generated, 
# and make graph to evaluate different schedulers.
#
# @author Yunsang Cho
# @author Hokeun Kim


import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import matplotlib
matplotlib.use('tkagg')

import os
import sys
import subprocess
import shutil
from functools import partial
import statistics

time_units = ['sec', 'msec', 'usec', 'nsec']

def step(num_workers, scheduler_type):

    global char_to_replace, template
    char_to_replace['$SCHEDULER_TYPE$'] = scheduler_type
    char_to_replace['$NUM_WORKERS$'] = str(num_workers)

    contents = template
    for key, value in char_to_replace.items():
        contents = contents.replace(key, value)

    filename = f'{"sporadic" if periodicity.get() == 1 else "periodic"}_{scheduler_type}_{num_workers}'
    filepath = f'{WORKING_DIR}/.gui/src/{filename}.lf'
    with open(filepath, "w") as lf_file:
        lf_file.write(contents)
        lf_file.close()
        print(f"File saved: {filepath}")

    out = subprocess.run([f'./gradlew','runLfc', '--args', filepath], capture_output=True)
    built_success = False
    for line in reversed(out.stdout.decode("utf-8").split('\n')):
        if line.startswith("BUILD SUCCESSFUL"):
            built_success = True
            break

    if built_success:
        print("Built Successfully!")
    else:
        print("Build Failed")
        print(out.stderr.decode("utf-8"))
        exit(0)

    lf_out = subprocess.run([f'{WORKING_DIR}/.gui/bin/{filename}'], capture_output=True)

    for line in reversed(lf_out.stdout.decode("utf-8").split('\n')):
        if line.startswith("---- Elapsed physical"):
            exe_time = line.split(' ')[-1]
            break

    exe_time = int(exe_time.replace(',','')) / 1000000000
    return exe_time
    

# A function called when the [Run] button is clicked
def runLfc():

    global char_to_replace, template
    workers = [1, 2, 5, 20]
    schedulers = {"NP": is_NP.get(), "GEDF_NP": is_GEDF_NP.get(), "GEDF_NP_CI": is_GEDF_NP_CI.get()}
    

    char_to_replace = {
        '$SCHEDULER_TYPE$': '',
        '$NUM_TASKS$': str(num_tasks.get()),
        '$TOTAL_TIME$': str(total_time.get()) + " " + str(total_time_unit.get()),
        '$UTILIZATION$': utilization.get(),
        '$NUM_WORKERS$': '',
        '$PERIODIC$':'false' if periodicity.get() == 1 else 'true',
        '$PERIOD$': str(period.get()) + " " + str(period_unit.get())
    }

    with open(TEMPLATE_PATH) as f:
        template = f.read()


    target_schedulers = [k for k, v in schedulers.items() if v == True]
    exe_times = {}

    workers = [i for i in range(1, 21)]

    
    for scheduler in target_schedulers:
        exe_time = []
        for worker in workers:
            exe_time.append(statistics.mean([step(worker, scheduler) for _ in range(num_iterations.get())]))
        exe_times[scheduler] = exe_time.copy()
        exe_time.clear()

    result = {}
    result['workers'] = workers
    result['exe_times'] = exe_times
    
    plot_graph(result)
    
def plot_graph(data):
    
    fig, ax = plt.subplots()
    plt.axis([1, 25, 0.0, 5.0])

    workers = data['workers']
    scatters = [1, 2, 5, 20]

    axes = []
    patches = []
    schedulers = data['exe_times'].keys()

    colors = ['#D81B60', '#1E88E5', '#FFC107', '#004D40', '#8794DD']
    for i, scheduler in enumerate(schedulers):
        
        patches.append(mpatches.Patch(color=colors[i], label=scheduler))

        if (graph_option.get() == 'line'):
            axes.append(ax.plot(workers, data['exe_times'][scheduler], '--', color=colors[i]))
        elif (graph_option.get() == 'dot'):
            axes.append(ax.scatter(workers, data['exe_times'][scheduler], color=colors[i]))

        #plt.scatter(scatters, [data['exe_times'][scheduler][i-1] for i in scatters], color=colors[i])
        
    ax.legend(handles=patches, loc='upper right')
    plt.axis([0, max(workers)+1, 0, max(max(i) for i in data['exe_times'].values()) * 1.2])
    
    plt.xlabel('Number of Worker')
    plt.ylabel('Execution time')
    title = ''
    if periodicity.get() == 1:
        title += 'Sporadic'
    else:
        title += f'Periodic(period: {char_to_replace["$PERIOD$"]})'
    title += f' / Total time: {char_to_replace["$TOTAL_TIME$"]} / Number of Tasks: {char_to_replace["$NUM_TASKS$"]} / Utilization: {char_to_replace["$UTILIZATION$"]}'

    plt.title(title, fontsize= 10)
    plt.show()

def create_input_frame(container):
    initialize()

    frame = ttk.Frame(container)

    frame.columnconfigure(0, weight=1)
    frame.columnconfigure(0, weight=3)

    ttk.Label(frame, text="Number of Tasks:").grid(column=0, row=0, sticky=tk.W)
    num_tasks_entry = ttk.Entry(frame, textvariable=num_tasks)
    num_tasks_entry.grid(column=1, row=0, sticky=tk.W)
    
    ttk.Label(frame, text="Total time:").grid(column=0, row=1, sticky=tk.W)
    total_time_entry = ttk.Entry(frame, textvariable=total_time)
    total_time_entry.grid(column=1, row=1, sticky=tk.W)

    select_time_unit = ttk.Combobox(frame, textvariable=total_time_unit, values=time_units, state='readonly')
    select_time_unit.current(0)
    select_time_unit.grid(column=2, row=1, sticky=tk.W)

    ttk.Label(frame, text="Utilization:").grid(column=0, row=2, sticky=tk.W)
    utilization_entry = ttk.Entry(frame, textvariable=utilization)
    utilization_entry.grid(column=1, row=2, sticky=tk.W)
    
    ttk.Label(frame, text="Number of iterations:").grid(column=0, row=3, sticky=tk.W)
    num_iterations_entry = ttk.Entry(frame, textvariable=num_iterations)
    num_iterations_entry.grid(column=1, row=3, sticky=tk.W)

    ttk.Label(frame, text="Periodicity:").grid(column=0, row=4, sticky=tk.W)
    ttk.Radiobutton(frame, text="sporadic", variable=periodicity, value=1).grid(column=1, row=4)
    ttk.Radiobutton(frame, text="periodic", variable=periodicity, value=2).grid(column=2, row=4)

    ttk.Label(frame, text="Period:").grid(column=0, row=5, sticky=tk.W)
    period_entry = ttk.Entry(frame, textvariable=period)
    period_entry.grid(column=1, row=5, sticky=tk.W)

    select_period_unit = ttk.Combobox(frame, textvariable=period_unit, values=time_units, state='readonly')
    select_period_unit.current(1)
    select_period_unit.grid(column=2, row=5, sticky=tk.W)

    scheduler_label = ttk.LabelFrame(frame, text="Scheduler")
    scheduler_label.grid(column=0, row=6, sticky=tk.W)

    ttk.Checkbutton(scheduler_label, text='NP', variable=is_NP, onvalue=True, offvalue=False).grid(column=0, row=0, sticky=tk.W)
    ttk.Checkbutton(scheduler_label, text='GEDF_NP', variable=is_GEDF_NP, onvalue=True, offvalue=False).grid(column=1, row=0, sticky=tk.W)
    ttk.Checkbutton(scheduler_label, text='GEDF_NP_CI', variable=is_GEDF_NP_CI, onvalue=True, offvalue=False).grid(column=2, row=0, sticky=tk.W)

    select_graph_option = ttk.LabelFrame(frame, text="Graph Option")
    select_graph_option.grid(column=2, row=6, sticky=tk.W)
    ttk.Radiobutton(select_graph_option, text="line", variable=graph_option, value="line").grid(column=0, row=0)
    ttk.Radiobutton(select_graph_option, text="dot", variable=graph_option, value="dot").grid(column=1, row=0)


    return frame


def exit(container):
    container.quit()

def set_global_variables():
    global num_tasks, total_time, total_time_unit, utilization, periodicity, period, period_unit, num_iterations
    global is_NP, is_GEDF_NP, is_GEDF_NP_CI, graph_option

    num_tasks = tk.IntVar()
    total_time = tk.IntVar()
    total_time_unit = tk.StringVar()
    utilization = tk.StringVar()
    num_iterations = tk.IntVar()
    periodicity = tk.IntVar()
    period = tk.IntVar()
    period_unit = tk.StringVar()
    is_NP = tk.BooleanVar()
    is_GEDF_NP = tk.BooleanVar()
    is_GEDF_NP_CI = tk.BooleanVar()
    graph_option = tk.StringVar()

    num_tasks.set(20)
    total_time.set(1)
    utilization.set("0.6")
    num_iterations.set(1)
    periodicity.set(1)
    period.set(100)
    is_NP.set(True)
    is_GEDF_NP.set(True)
    is_GEDF_NP_CI.set(True)
    graph_option.set("line")


def create_main():

    main = tk.Tk()
    main.title('TaskSet generator')
    main.geometry('600x400')
    main.resizable(False, False)

    main.rowconfigure(0, weight=3)
    main.rowconfigure(1, weight=1)

    set_global_variables()

    input_frame = create_input_frame(main)
    input_frame.grid(column=0, row=0, sticky="NSEW")

    button_frame = ttk.Frame(main)
    button_frame.grid(column=0, row=1, sticky=tk.W)

    runButton = ttk.Button(button_frame, text="Run", command=runLfc)
    runButton.grid(column=0, row=0)

    exitButton = ttk.Button(button_frame, text='Exit', command=partial(exit, main))
    exitButton.grid(column=1, row=0)

    main.mainloop()

def initialize():
    if (os.path.isdir(f'{WORKING_DIR}/.gui')):
        shutil.rmtree(f'{WORKING_DIR}/.gui')

    os.mkdir(f'{WORKING_DIR}/.gui')
    os.mkdir(f'{WORKING_DIR}/.gui/src')

if __name__ == "__main__":
    global LF_PATH, WORKING_DIR, TEMPLATE_PATH
    LF_PATH = os.getenv("LF_PATH")
    if LF_PATH == None:
        sys.exit("Set the environment variable LF_PATH to the path where Lingua Franca is installed")
    
    WORKING_DIR = os.getcwd()
    TEMPLATE_PATH = f'{WORKING_DIR}/BasicTaskSetGeneratorTemplate.lf'
    os.chdir(LF_PATH)
    create_main()
