

import numpy as np

import os
import sys
import subprocess
import shutil
from functools import partial
import statistics

def saveFile():

    global template
    
    with open(TEMPLATE_PATH) as f:
        template = f.read()
         
    char_to_replace= {
        '$TIMEOUT$':'10 sec',
        '$NUM_WORKERS$':'8',
        '$STARTOUTPUT$' : 'output out0:time;\n\
    output out1:time;\n\
    output out2:time;',
        '$STARTUPREACTION$': 'reaction(startup) -> out0, out1, out2 {=\n\
        lf_set(out0, self->exe_time);\n\
        lf_set(out1, self->exe_time);\n\
        lf_set(out2, self->exe_time);\n\
    =}',
        #'$TASKCONFIG$': '',
    }
    
    contents = template
    for key, value in char_to_replace.items():
        contents = contents.replace(key, value)
    
    filename = f'{"Dag"}_{"Test"}'
    filepath = f'{WORKING_DIR}/.gui/src/{filename}.lf'
    with open(filepath, "w") as lf_file:
        lf_file.write(contents)
        lf_file.close()
        print(f"File saved: {filepath}")

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
    saveFile()
