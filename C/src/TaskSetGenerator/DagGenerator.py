# DAG taskset generator
# @author Yunsang Cho
# @author ByeongGil Jun

from copyreg import constructor
import numpy as np

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


def saveFile():

    global char_to_replace, template
    
    with open(TEMPLATE_PATH) as f:
        template = f.read()

    
    contents = template
    for key, value in char_to_replace.items():
        contents = contents.replace(key, value)
    
    filename = f'{"Dag"}_{"Test"}'
    filepath = f'{WORKING_DIR}/.gui/src/{filename}.lf'
    with open(filepath, "w") as lf_file:
        lf_file.write(contents)
        lf_file.close()
        print(f"File saved: {filepath}")


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
    random.seed(seed.timestamp())

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

    if (os.path.isdir(f'{WORKING_DIR}/.gui')):
        shutil.rmtree(f'{WORKING_DIR}/.gui')

    os.mkdir(f'{WORKING_DIR}/.gui')
    os.mkdir(f'{WORKING_DIR}/.gui/src')
    str_Generator(num_outputs=4, max_depth=4, seed=datetime.now())
    saveFile()



