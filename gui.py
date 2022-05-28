import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np

import matplotlib
matplotlib.use('tkagg')

import os
import subprocess
import shutil
from functools import partial

TEMPLATE_PATH = '../hyu-iot/examples-lingua-franca/C/src/TaskSetGenerator/BasicTaskSetGeneratorTemplate.lf'


class TaskGenerator:
    def __init__(self, template_path):

        self.gui = tk.Tk()

        with open(template_path) as f:
            self.template = f.read()

        self.char_to_replace = {
            'SCHEDULER_TYPE': 'NP',
            'NUM_TASKS': '20',
            'TOTAL_TIME': '1',
            'UTILIZATION': '0.6',
            'NUM_WORKERS': '8'
        }


    def step(self, num_workers, scheduler_type):
        self.template['SCHEDULER_TYPE'] = scheduler_type
        self.template['NUM_WORKERS'] = str(num_workers)

        contents = self.template
        for key, value in self.char_to_replace.items():
            contents = contents.replace(key, value)

        filename = f'practice_{scheduler_type}_{num_workers}'
        filepath = f'./.gui/src/{filename}.lf'
        with open(filepath, "w") as lf_file:
            lf_file.write(contents)
            lf_file.close()
            print(f"File saved: {filepath}")

        out = subprocess.run(['./gradlew','runLfc', '--args', filepath], capture_output=True)

        print("Build sccessfully!")
        lf_out = subprocess.run([f'./.gui/bin/{filename}'], capture_output=True)

        for line in reversed(lf_out.stdout.decode("utf-8").split('\n')):
            if line.startswith("---- Elapsed physical"):
                exe_time = line.split(' ')[-1]
                break

        exe_time = int(exe_time.replace(',',''))
        return exe_time


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

    out = subprocess.run(['./gradlew','runLfc', '--args', filepath], capture_output=True)

    print("Build sccessfully!")
    lf_out = subprocess.run([f'./.gui/bin/{filename}'], capture_output=True)

    for line in reversed(lf_out.stdout.decode("utf-8").split('\n')):
        if line.startswith("---- Elapsed physical"):
            exe_time = line.split(' ')[-1]
            break

    exe_time = int(exe_time.replace(',','')) / 1000000
    return exe_time
    




def runLfc():
    print(numOfTasks.get())

    global char_to_replace, template
    workers = [1, 2, 5, 20]
    schedulers = ["NP", "GEDF_NP", "GEDF_NP_CI"]
    

    char_to_replace = {
        'SCHEDULER_TYPE': 'NP',
        'NUM_TASKS': str(numOfTasks.get()),
        'TOTAL_TIME': str(totalTime.get()),
        'UTILIZATION': utilization.get(),
        'NUM_WORKERS': '8'
    }


    with open(TEMPLATE_PATH) as f:
        template = f.read()

    exe_times = {}

    for scheduler in schedulers:
        exe_time = []
        for worker in range(1, 21):
            exe_time.append(step(worker, scheduler))
        exe_times[scheduler] = exe_time.copy()
        exe_time.clear()

    result = {}
    result['workers'] = [i for i in range(1, 21)]
    result['exe_times'] = exe_times
    
    plot_graph(result)
    #print(result)


def plot_graph(data):
    
    plt.axis([1, 25, 0.0, 5.0])

    workers = data['workers']
    scatters = [1, 2, 5, 20]
    n = 1

    for scheduler in data['exe_times'].keys():
        
        colors = np.random.rand(n)
        n += 1
        
        plt.plot(workers, data['exe_times'][scheduler], '--')
        plt.scatter(scatters, [data['exe_times'][scheduler][i-1] for i in scatters])
        #plt.plot(X, workers, 'o', color='g')
        #plt.plot(X, data['exe_times'][scheduler], '--')


    plt.show()

def create_input_frame(container):
    frame = ttk.Frame(container)

    frame.columnconfigure(0, weight=1)
    frame.columnconfigure(0, weight=3)

    ttk.Label(frame, text="Number of Tasks:").grid(column=0, row=0, sticky=tk.W)
    numOfTasks_entry = ttk.Entry(frame, textvariable=numOfTasks)
    numOfTasks_entry.grid(column=1, row=0, sticky=tk.W)
    

    ttk.Label(frame, text="Total time:").grid(column=0, row=1, sticky=tk.W)
    totalTime_entry = ttk.Entry(frame, textvariable=totalTime)
    totalTime_entry.grid(column=1, row=1, sticky=tk.W)
    

    ttk.Label(frame, text="Utilization:").grid(column=0, row=2, sticky=tk.W)
    utilization_entry = ttk.Entry(frame, textvariable=utilization)
    utilization_entry.grid(column=1, row=2, sticky=tk.W)
    
    # numOfTasks_entry.pack()
    # totalTime_entry.pack()
    # utilization_entry.pack()

    return frame


def exit(container):
    container.quit()


def create_main():
    global numOfTasks, totalTime, utilization

    main = tk.Tk()
    main.title('TaskSet generator')
    main.geometry('600x400')
    main.resizable(False, False)

    main.rowconfigure(0, weight=3)
    main.rowconfigure(1, weight=1)

    numOfTasks = tk.IntVar()
    totalTime = tk.IntVar()
    utilization = tk.StringVar()

    numOfTasks.set(20)
    totalTime.set(1)
    utilization.set("0.6")

    input_frame = create_input_frame(main)
    input_frame.grid(column=0, row=0, sticky="NSEW")

    runButton = ttk.Button(main, text="Run", command=runLfc)
    runButton.grid(column=0, row=1)

    exitButton = ttk.Button(main, text='Exit', command=partial(exit, main))
    exitButton.grid(column=1, row=1)
    
    main.mainloop()


if __name__ == "__main__":

    if (os.path.isdir('./.gui')):
        shutil.rmtree('./.gui')
    os.mkdir('./.gui')
    os.mkdir('./.gui/src')
    create_main()