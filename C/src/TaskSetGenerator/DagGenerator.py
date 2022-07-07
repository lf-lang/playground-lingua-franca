

import numpy as np

import os
import sys
import subprocess
import shutil
import random
from functools import partial
from datetime import datetime
import statistics


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

def task_config(num_outputs, max_depth=4, seed=datetime.now()):
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

def str_Generator(num_outputs) :
    global char_to_replace
             
    char_to_replace= {
        '$TIMEOUT$':'10 sec',
        '$NUM_WORKERS$':'8',
        '$STARTOUTPUT$' : '',
        '$STARTUPREACTION$': '',
        '$TASKCONFIG$': '',
    }

    outputs = [f'out{i}' for i in range(num_outputs)]

    for out in outputs:
        char_to_replace['$STARTOUTPUT$'] += f'output {out}:time;\n\t'

    char_to_replace['$STARTUPREACTION$'] += "reaction(startup) -> " + ", ".join(outputs) + " {=\n"
    for out in outputs:
        char_to_replace['$STARTUPREACTION$'] += f"\t\tlf_set({out}, self->exe_time);\n"
    char_to_replace['$STARTUPREACTION$'] += "\t=}"

    char_to_replace['$TASKCONFIG$'] += task_config(num_outputs, max_depth=8)


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
    str_Generator(8)
    saveFile()
