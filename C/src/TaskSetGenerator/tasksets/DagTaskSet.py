# DAG TaskSet
# @author Yunsang Cho
# @author ByeongGil Jun

import numpy as np
import os
import sys
import random
from datetime import datetime


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


class TaskSet(object):

    def __init__(self, TEMPLATE_PATH=''):
        self.config = {
            'schedulers': ['NP'],
            'type': 'basic',
            'timeout': {'value': 10, 'timeUnit': 'sec'},
            'min_workers': 1,
            'max_workers': 20,
            'seed': datetime.now(),
            'max_depth': 4,
            'num_outputs': 4,
            'execution_time': {'value': 100, 'timeUnit': 'msec'}
        }

        if os.path.isfile(TEMPLATE_PATH) == True:
            with open(TEMPLATE_PATH) as f:
                self.template = f.read()
        else:
            sys.exit('The template path is invalid.')

    def setConfig(self, config):
        for key, value in config.items():
            if key in self.config.keys():
                self.config[key] = value


    def makeLF(self, saveDir='./'):
        if os.path.isdir(saveDir) == False:
            sys.exit("The directory for saving result is not exist.")
        
        char_to_replace = {
            '$STARTOUTPUT$' : '',
            '$STARTUPREACTION$': '',
            '$TASKCONFIG$': '',
            '$COMPONENTS$': '',
            '$EXE_TIME$':'',
        }

        char_to_replace['$TIMEOUT$'] = f'{self.config["timeout"]["value"]} {self.config["timeout"]["timeUnit"]}'
        
        outputs = [f'out{i}' for i in range(self.config['num_outputs'])]

        for out in outputs:
            char_to_replace['$STARTOUTPUT$'] += f'output {out}:time;\n\t'

        char_to_replace['$STARTUPREACTION$'] += "reaction(startup) -> " + ", ".join(outputs) + " {=\n"
        for out in outputs:
            char_to_replace['$STARTUPREACTION$'] += f"\t\tlf_set({out}, self->exe_time);\n"
        char_to_replace['$STARTUPREACTION$'] += "\t=}"
        char_to_replace['$COMPONENTS$'] += self.make_component_reactor()

        char_to_replace['$TASKCONFIG$'] += self.task_config_multiple_inputs(seed=self.config['seed'])

        char_to_replace['$EXE_TIME$'] += f'{self.config["execution_time"]["value"]} {self.config["execution_time"]["timeUnit"]}'

        workers = [w for w in range(self.config['min_workers'], self.config['max_workers']+1)]

        generated_files = {
            'workers': workers,
            'schedulers': {
                'NP': [],
                'GEDF_NP': [],
                'GEDF_NP_CI': [],
            },
        }
        
        for scheduler in self.config['schedulers']:
            char_to_replace['$SCHEDULER_TYPE$'] = scheduler
            for worker in workers:
                char_to_replace['$NUM_WORKERS$'] = str(worker)
                FILE_NAME = f'DAG_{scheduler}_{worker}.lf'
                FILE_PATH = f'{saveDir}/{FILE_NAME}'

                contents = self.template
                for k, v in char_to_replace.items():
                    contents = contents.replace(k, v)
                
                with open(FILE_PATH, 'w') as lf_file:
                    lf_file.write(contents)
                    lf_file.close()
                
                if os.path.isfile(FILE_PATH) == True:
                    print(f'File saved: {FILE_PATH}')
                    generated_files['schedulers'][scheduler].append(FILE_PATH)
                else:
                    sys.exit('The LF file is not generated.')
        
        return generated_files                

    def make_component_reactor(self):
        result = ''
        for i in range(1, self.config['num_outputs']+1):
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


    def task_config_multiple_inputs(self, seed=datetime.now()):
        random.seed(seed)

        heights = [random.randint(1, self.config['num_outputs']) for _ in range(self.config['max_depth'])]
        heights[0] = self.config['num_outputs']

        arr_for_random = [[i for i in range(N)] for N in range(1, self.config['num_outputs']+1)]
        task_arr = [[] for _ in range(self.config['max_depth'])]

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
