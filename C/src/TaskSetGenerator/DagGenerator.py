

import numpy as np

import os
import sys
import subprocess
import shutil
from functools import partial
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

def str_Generator(num_outputs) :
    global char_to_replace
    backspace = '\b'
             
    char_to_replace= {
        '$TIMEOUT$':'10 sec',
        '$NUM_WORKERS$':'8',
        '$STARTOUTPUT$' : '',
        '$STARTUPREACTION$': '',
        #'$TASKCONFIG$': '',
    }

    char_to_replace['$STARTUPREACTION$']+=f'{"reaction(startup) ->"}'
    for i in range(0, num_outputs-1) :
        char_to_replace['$STARTUPREACTION$']+=f'{" out"}{i}{","}'
    char_to_replace['$STARTUPREACTION$']+=f'{" out"}{num_outputs-1}'
    for i in range(0, num_outputs) :
        if (i == 0) :
            char_to_replace['$STARTUPREACTION$']+=' {=\n\t\t'
        char_to_replace['$STARTOUTPUT$']+=f'{"output out"}{i}{":time;"}{chr(10)}{chr(9)}'
        char_to_replace['$STARTUPREACTION$']+=f'{"lf_set(out"}{i}{", self->exe_time);"}'
        if (i != num_outputs-1) :
            char_to_replace['$STARTUPREACTION$']+=f'{chr(10)}{chr(9)}{chr(9)}'
        else :
            char_to_replace['$STARTUPREACTION$']+=f'{chr(10)}{chr(9)}'
    char_to_replace['$STARTUPREACTION$']+='=}'


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
    str_Generator(3)
    saveFile()
