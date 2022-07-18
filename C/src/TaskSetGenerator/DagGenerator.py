# DAG taskset generator
# @author Yunsang Cho
# @author ByeongGil Jun

from copyreg import constructor


import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import matplotlib
matplotlib.use('tkagg')

import os
import sys
import subprocess
import shutil
import random
from functools import partial
from datetime import datetime
import statistics


# A node corresponding to one DAG component reactor
class Node:
    
    def __init__(self, id, parents):
        self.id = id
        self.level = int(str(id).split('_')[0])
        self.index = int(str(id).split('_')[-1])
        self.name = f'task_{id}'
        if self.level > 1:
            self.parents = [f'task_{self.level-1}_{p}' for p in sorted(parents)]
        else:
            self.parents = ['runner']
        self.size= len(self.parents)

    def get_string(self):
        result = f'{self.name} = new Component_{self.size}();\n'
        if self.level == 1:
            return result + f'\trunner.out{self.index} -> {self.name}.in1;\n\n'


        for i, parent in enumerate(self.parents):
            result += f'\t{parent}.out -> {self.name}.in{i+1};\n'
        
        return result + '\n'


def step(num_workers, scheduler_type):

    global char_to_replace, template
    char_to_replace['$SCHEDULER_TYPE$'] = scheduler_type
    char_to_replace['$NUM_WORKERS$'] = str(num_workers)

    contents = template
    for key, value in char_to_replace.items():
        contents = contents.replace(key, value)
    
    filename = f'DAG_{num_workers}'
    filepath = f'{WORKING_DIR}/.gui/src/{filename}.lf'
    with open(filepath, "w") as lf_file:
        lf_file.write(contents)
        lf_file.close()
        if (os.path.exists(filepath)):
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

def plot_graph(data):
    
    fig, ax = plt.subplots()
    plt.axis([1, 25, 0.0, 5.0])

    workers = data['workers']
    scatters = [1, 2, 5, 20]

    axes = []
    patches = []
    schedulers = data['exe_times'].keys()

    colors = ['#D81B60', '#1E88E5', '#FFC107', '#004D40', '#8794DD']
    
    patches.append(mpatches.Patch(color=colors[0], label='NP'))

    axes.append(ax.plot(workers, data['exe_times']['NP'], '--', color=colors[0]))
    

    ax.legend(handles=patches, loc='upper right')
    plt.axis([0, max(workers)+1, 0, max(max(i) for i in data['exe_times'].values()) * 1.2])
    
    plt.xlabel('Number of Worker')
    plt.ylabel('Execution time')
    title = ''
    title += 'DAG_NP_2468'

    plt.title(title, fontsize= 10)
    plt.show()

def run():

    global char_to_replace, template
    
    with open(TEMPLATE_PATH) as f:
        template = f.read()

    workers = [i for i in range(1, 21)]
    contents = template
    exe_times = {}
    exe_times['NP'] = []
    for num_worker in workers:
        exe_time = step(num_worker, 'NP')
        exe_times['NP'].append(exe_time)
    
    result = {}
    result['workers'] = workers
    result['exe_times'] = exe_times

    plot_graph(result)


def task_config_simple(num_outputs, max_depth=4, seed=datetime.now()):
    random.seed(seed)
    tasks = ''
    depths = []

    depths.append(max_depth)    # The first node has a depth to the end.
    for i in range(1, num_outputs):
        depths.append(random.randint(1, max_depth))
    
    print(f'Depths: {depths}')

    # Configure nodes at level 1.
    for i in range(num_outputs):
        tasks += f'task_1_{i} = new Component();\n\t'
    tasks += '\n\t'
    for i in range(num_outputs):
        tasks += f'runner.out{i} -> task_1_{i}.in;\n\t'
    tasks += '\n'

    # Configure level 2 or higher nodes
    current_level = 1
    while current_level < max_depth:
        for i in range(num_outputs):
            if depths[i] > current_level:
                tasks += f'\ttask_{current_level+1}_{i} = new Component();\n'
        tasks += '\n'
        for i in range(num_outputs):
            if depths[i] > current_level:
                tasks += f'\ttask_{current_level}_{i}.out -> task_{current_level+1}_{i}.in;\n'
        tasks += '\n'
        current_level += 1

    return tasks


def task_config_multiple_inputs(num_outputs, max_depth, seed=datetime.now()):
    random.seed(seed)
    #random.seed(seed.timestamp())

    heights = [random.randint(1, num_outputs) for _ in range(max_depth)]
    heights[0] = num_outputs

    arr_for_random = [[i for i in range(N)] for N in range(1, num_outputs+1)]
    task_arr = [[] for _ in range(max_depth)]

    for i, height in enumerate(heights):
        if i == 0:
            for h in range(height):
                task_arr[i].append(Node(f'{i+1}_{h}', []))
        else:
            sizes = [random.randint(1, heights[i-1]) for _ in range(height)]
            for h in range(height):
                random.shuffle(arr_for_random[heights[i-1]-1])
                task_arr[i].append(Node(f'{i+1}_{h}', arr_for_random[heights[i-1]-1][:sizes[h]].copy()))

    task_config = ''
    for tasks in task_arr:
        for task in tasks:
            task_config += task.get_string() + '\t'

    return task_config


    

def make_component_reactor(num_outputs=1):
    result = ''
    for i in range(1, num_outputs+1):
        inputs = [f'in{j}' for j in range(1, i+1)]
        
        component = f'reactor Component_{i} {{\n'
                
        for input in inputs:
            component += f'\tinput {input}:time;\n'
        component += f'\toutput out:time;\n'
        component += f'\tstate exe_time:time(0);\n\n'
        
        # define reaction
        component += f'\treaction({", ".join(inputs)}) -> out {{=\n'
        component += f'\t\tlong long int physical_start_time = lf_time_physical();\n\n'
        component += f'\t\tif (in1->is_present) {{\n\t\t\tself->exe_time = in1->value;\n\t\t}}'
        
        for input in inputs[1:]:
            component += f' else if ({input}->is_present) {{\n\t\t\tself->exe_time = {input}->value;\n\t\t}}'

        component += '\n\t\twhile (lf_time_physical() < physical_start_time + self->exe_time) {\n\n\t\t}\n'
        component += '\t\tlf_set(out, self->exe_time);\n'
        component += '\t=}\n}\n'

        result += component + '\n'

    return result


def str_Generator(num_outputs, max_depth, seed=datetime.now()) :
    global char_to_replace
             
    char_to_replace= {
        '$TIMEOUT$':'10 sec',
        '$NUM_WORKERS$':'8',
        '$STARTOUTPUT$' : '',
        '$STARTUPREACTION$': '',
        '$TASKCONFIG$': '',
        '$COMPONENTS$': '',
        '$SCHEDULER_TYPE$': 'NP',
        '$EXE_TIME$': '200 msec'
    }

    outputs = [f'out{i}' for i in range(num_outputs)]

    for out in outputs:
        char_to_replace['$STARTOUTPUT$'] += f'output {out}:time;\n\t'

    char_to_replace['$STARTUPREACTION$'] += "reaction(startup) -> " + ", ".join(outputs) + " {=\n"
    for out in outputs:
        char_to_replace['$STARTUPREACTION$'] += f"\t\tlf_set({out}, self->exe_time);\n"
    char_to_replace['$STARTUPREACTION$'] += "\t=}"
    char_to_replace['$COMPONENTS$'] += make_component_reactor(num_outputs)


    #char_to_replace['$TASKCONFIG$'] += task_config_simple(num_outputs, max_depth=8)
    char_to_replace['$TASKCONFIG$'] += task_config_multiple_inputs(num_outputs, max_depth, seed)

   


    
if __name__ == "__main__":
    global LF_PATH, WORKING_DIR, TEMPLATE_PATH
    LF_PATH = os.getenv("LF_PATH")
    if LF_PATH == None:
        sys.exit("Set the environment variable LF_PATH to the path where Lingua Franca is installed")
    
    WORKING_DIR = os.getcwd()
    TEMPLATE_PATH = f'{WORKING_DIR}/DagTaskSetGeneratorTemplate.lf'
    os.chdir(LF_PATH)

    # if (os.path.isdir(f'{WORKING_DIR}/.gui')):
    #     shutil.rmtree(f'{WORKING_DIR}/.gui')

    # os.mkdir(f'{WORKING_DIR}/.gui')
    # os.mkdir(f'{WORKING_DIR}/.gui/src')
    str_Generator(num_outputs=4, max_depth=4, seed=2468)
    run()
    



