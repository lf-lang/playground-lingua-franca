# Basic Taskset
# @author Yunsang Cho
# @author Hokeun Kim

import os
import sys

class TaskSet(object):

    def __init__(self, TEMPLATE_PATH=''):
        self.config = {
            'schedulers': ['NP'],
            'periodicity': 'sporadic',
            'period': {'value': 0, 'timeUnit': 'sec'},
            'timeout': {'value': 10, 'timeUnit': 'sec'},
            'min_workers': 1,
            'max_workers': 20,
            'num_tasks': 20,
            'utilization': 0.6,
            'seed': 0
        }

        if os.path.isfile(TEMPLATE_PATH) == True:
            with open(TEMPLATE_PATH) as f:
                self.template = f.read()
        else:
            raise RuntimeError('Invalid template path: ' + TEMPLATE_PATH)

    def setConfig(self, config):
        for key, value in config.items():
            if key in self.config.keys():
                self.config[key] = value


    def makeLF(self, outputDir='./'):
        if not os.path.isdir(outputDir):
            raise RuntimeError("No output directory: " + outputDir)
        
        char_to_replace = {}
        char_to_replace['$TOTAL_TIME$'] = f'{self.config["timeout"]["value"]} {self.config["timeout"]["timeUnit"]}'
        char_to_replace['$UTILIZATION$'] = str(self.config['utilization'])
        char_to_replace['$PERIODIC$'] = 'true' if self.config['periodicity'] == 'periodic' else 'false'
        char_to_replace['$PERIOD$'] = f'{self.config["period"]["value"]} {self.config["period"]["timeUnit"]}'
        char_to_replace['$NUM_TASKS$'] = str(self.config['num_tasks'])
        char_to_replace['$RANDOM_SEED$'] = str(self.config['seed'])

        workers = [w for w in range(self.config['min_workers'], self.config['max_workers']+1)]
        generated_files = {
            'workers': workers,
            'schedulers': {
                'NP': [],
                'GEDF_NP': [],
                'GEDF_NP_CI': [],
                'adaptive': [],
            },
        }

        for scheduler in self.config['schedulers']:
            char_to_replace['$SCHEDULER_TYPE$'] = scheduler
            for worker in workers:
                char_to_replace['$NUM_WORKERS$'] = str(worker)
                FILE_NAME = f'{self.config["periodicity"].capitalize()}_{scheduler}_{worker}.lf'
                FILE_PATH = f'{outputDir}/{FILE_NAME}'

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
                    raise RuntimeError('Failed to generate LF file: ' + FILE_PATH)

        return generated_files
