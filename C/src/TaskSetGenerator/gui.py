import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.interpolate import interp1d
import numpy as np

import matplotlib
matplotlib.use('tkagg')

import os
import sys
import subprocess
import shutil
from functools import partial
import random

TEMPLATE_PATH = './BasicTaskSetGeneratorTemplate.lf'



def step(num_workers, scheduler_type):

    global char_to_replace, template
    char_to_replace['SCHEDULER_TYPE'] = scheduler_type
    char_to_replace['NUM_WORKERS'] = str(num_workers)

    contents = template
    for key, value in char_to_replace.items():
        contents = contents.replace(key, value)

    filename = f'practice_{scheduler_type}_{num_workers}'
    filepath = f'./.gui/src/{filename}.lf'
    with open(filepath, "w") as lf_file:
        lf_file.write(contents)
        lf_file.close()
        print(f"File saved: {filepath}")

    out = subprocess.run([f'{LF_PATH}/gradlew','runLfc', '--args', filepath], capture_output=True)

    print("Build sccessfully!")
    lf_out = subprocess.run([f'./.gui/bin/{filename}'], capture_output=True)

    for line in reversed(lf_out.stdout.decode("utf-8").split('\n')):
        if line.startswith("---- Elapsed physical"):
            exe_time = line.split(' ')[-1]
            break

    exe_time = int(exe_time.replace(',','')) / 1000000000
    return exe_time
    




def runLfc():

    global char_to_replace, template
    workers = [1, 2, 5, 20]
    schedulers = ["NP", "GEDF_NP", "GEDF_NP_CI"]
    

    char_to_replace = {
        'SCHEDULER_TYPE': 'NP',
        'NUM_TASKS': str(numOfTasks.get()),
        'TOTAL_TIME': str(totalTime.get()) + " " + str(totalTimeUnit.get()),
        'UTILIZATION': utilization.get(),
        'NUM_WORKERS': '1',
        'PERIODIC':'false' if periodicity.get() == 1 else 'true',
        'PERIOD': str(period.get()) + " " + str(periodUnit.get())
    }

    with open(TEMPLATE_PATH) as f:
        template = f.read()

    exe_times = {}

    workers = [i for i in range(1, 21)]
    for scheduler in schedulers:
        exe_time = []
        for worker in workers:
            exe_time.append(step(worker, scheduler))
        exe_times[scheduler] = exe_time.copy()
        exe_time.clear()

    result = {}
    result['workers'] = workers
    result['exe_times'] = exe_times
    
    plot_graph(result)
    #print(result)


def plot_graph(data):
    
    fig, ax = plt.subplots()
    plt.axis([1, 25, 0.0, 5.0])

    workers = data['workers']
    scatters = [1, 2, 5, 20]

    axes = []
    patches = []
    schedulers = data['exe_times'].keys()
    for scheduler in schedulers:
        
        color = (random.random(), random.random(), random.random())
        patches.append(mpatches.Patch(color=color, label=scheduler))

        axes.append(ax.plot(workers, data['exe_times'][scheduler], '--', color=color))
        plt.scatter(scatters, [data['exe_times'][scheduler][i-1] for i in scatters], color=color)
        
    ax.legend(handles=patches, loc='upper right')
    plt.axis([0, max(workers)+1, 0, max(max(i) for i in data['exe_times'].values()) * 1.2])
    
    plt.xlabel('Number of Worker')
    plt.ylabel('Execution time')
    title = ''
    if periodicity.get() == 1:
        title += 'Sporadic'
    else:
        title += f'Periodic(period: {char_to_replace["PERIOD"]})'
    title += f' / Total time: {char_to_replace["TOTAL_TIME"]} / Number of Tasks: {char_to_replace["NUM_TASKS"]} / Utilization: {char_to_replace["UTILIZATION"]}'

    plt.title(title, fontsize= 10)
    #plt.title(f'{"Sporadic" if periodicity.get() == 1 else f"Periodic"} / Total time: {char_to_replace["TOTAL_TIME"]} / Number of Tasks: {char_to_replace["NUM_TASKS"]} / Utilization: {char_to_replace["UTILIZATION"]}')
    plt.show()

timeUnits = ['sec', 'msec', 'usec', 'nsec']

def create_input_frame(container):
    initialize()

    frame = ttk.Frame(container)

    frame.columnconfigure(0, weight=1)
    frame.columnconfigure(0, weight=3)

    ttk.Label(frame, text="Number of Tasks:").grid(column=0, row=0, sticky=tk.W)
    numOfTasks_entry = ttk.Entry(frame, textvariable=numOfTasks)
    numOfTasks_entry.grid(column=1, row=0, sticky=tk.W)
    

    ttk.Label(frame, text="Total time:").grid(column=0, row=1, sticky=tk.W)
    totalTime_entry = ttk.Entry(frame, textvariable=totalTime)
    totalTime_entry.grid(column=1, row=1, sticky=tk.W)

    select_timeUnit = ttk.Combobox(frame, textvariable=totalTimeUnit, values=timeUnits, state='readonly')
    select_timeUnit.current(0)
    select_timeUnit.grid(column=2, row=1, sticky=tk.W)

    ttk.Label(frame, text="Utilization:").grid(column=0, row=2, sticky=tk.W)
    utilization_entry = ttk.Entry(frame, textvariable=utilization)
    utilization_entry.grid(column=1, row=2, sticky=tk.W)
    
    ttk.Label(frame, text="Periodicity:").grid(column=0, row=3, sticky=tk.W)
    ttk.Radiobutton(frame, text="sporadic", variable=periodicity, value=1).grid(column=1, row=3)
    ttk.Radiobutton(frame, text="periodic", variable=periodicity, value=2).grid(column=2, row=3)
    #b2.pack(anchor= tk.W)

    ttk.Label(frame, text="Period:").grid(column=0, row=4, sticky=tk.W)
    period_entry = ttk.Entry(frame, textvariable=period)
    period_entry.grid(column=1, row=4, sticky=tk.W)

    select_periodUnit = ttk.Combobox(frame, textvariable=periodUnit, values=timeUnits, state='readonly')
    select_periodUnit.current(1)
    select_periodUnit.grid(column=2, row=4, sticky=tk.W)

    return frame


def exit(container):
    container.quit()


def create_main():
    global numOfTasks, totalTime, totalTimeUnit, utilization, periodicity, period, periodUnit

    main = tk.Tk()
    main.title('TaskSet generator')
    main.geometry('600x400')
    main.resizable(False, False)

    main.rowconfigure(0, weight=3)
    main.rowconfigure(1, weight=1)

    numOfTasks = tk.IntVar()
    totalTime = tk.IntVar()
    totalTimeUnit = tk.StringVar()
    utilization = tk.StringVar()
    periodicity = tk.IntVar()
    period = tk.IntVar()
    periodUnit = tk.StringVar()

    numOfTasks.set(20)
    totalTime.set(1)
    utilization.set("0.6")
    periodicity.set(1)
    period.set(100)

    input_frame = create_input_frame(main)
    input_frame.grid(column=0, row=0, sticky="NSEW")

    
    runButton = ttk.Button(main, text="Run", command=runLfc)
    runButton.grid(column=0, row=1)

    exitButton = ttk.Button(main, text='Exit', command=partial(exit, main))
    exitButton.grid(column=1, row=1)

    main.mainloop()

def initialize():
    if (os.path.isdir('./.gui')):
        shutil.rmtree('./.gui')

    os.mkdir('./.gui')
    os.mkdir('./.gui/src')


if __name__ == "__main__":
    global LF_PATH
    LF_PATH = os.getenv("LF_PATH")
    if LF_PATH == None:
        sys.exit("Set the environment variable LF_PATH to the path where Lingua Franca is installed")
        
    create_main()