# Basic Taskset
# @author Yunsang Cho
# @author Hokeun Kim


import os
import sys
import numpy as np

class BasicTaskSet(object):

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
            'seed': 0,
            'p_deadline': 0.6,
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
        char_to_replace['$TASKCONFIG$'] = self.task_config()

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
            print(self.config['schedulers'])
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

    def translate_TimeValue(self, TimeValue):
        TimeUnits = {
            'sec': 1000000000,
            'msec': 1000000,
            'usec': 1000,
            'nsec': 1
        }
        return TimeValue['value'] * TimeUnits[TimeValue['timeUnit']]

    def task_config(self):
        configs = ""

        deadlines = np.random.rand(self.config['num_tasks'])
        deadline_task_indexs = deadlines <= self.config['p_deadline']
        deadlines[~deadline_task_indexs] = 0

        total_time = self.translate_TimeValue(self.config['timeout'])

        if self.config['periodicity'] == 'sporadic':
            
            exe_time = int(total_time * self.config['utilization'] / float(self.config['num_tasks']))
            if self.config['p_deadline'] > 0 and deadlines.sum() > 0:
                ratio = (total_time - exe_time) / float(self.config['p_deadline'])
                print(f"sproadic -> ratio: {ratio}")
                deadlines[deadline_task_indexs] = deadlines[deadline_task_indexs] * ratio + exe_time
        
        elif self.config['periodicity'] == 'periodic':
            period = self.translate_TimeValue(self.config['period'])
            exe_time = int(period * self.config['utilization'] / float(self.config['num_tasks']))
            if self.config['p_deadline'] > 0 and deadlines.sum() > 0:
                ratio = (period - exe_time) / float(self.config['p_deadline'])
                print(f"periodic -> ratio: {ratio}")
                deadlines[deadline_task_indexs] = deadlines[deadline_task_indexs] * ratio + exe_time

        deadlines = np.array([int(d) for d in deadlines])
        np.random.shuffle(deadlines)
        deadline_task_indexs = deadlines > 0

        total_time = f"{self.config['timeout']['value']} {self.config['timeout']['timeUnit']}"
        exe_time = f"{exe_time} nsec"
        period = f"{self.config['period']['value']} {self.config['period']['timeUnit']}"
        periodicity = "true" if self.config['periodicity'] == 'periodic' else "false"
        release_time = f"{period}" if self.config['periodicity'] == 'periodic' else "0 nsec"

        for i, d in enumerate(deadlines):
            if d > 0:
                configs += f"\ttask{i} = new TaskWithDeadline(id={i}, release_time={release_time}, total_time={total_time}, exe_time={exe_time}, periodic={periodicity}, period={period}, deadline_time={d} nsec);\n"
            else: 
                configs += f"\ttask{i} = new TaskWithoutDeadline(id={i}, release_time={release_time}, total_time={total_time}, exe_time={exe_time}, periodic={periodicity}, period={period});\n"

        configs += "\n"

        for i in range(deadlines.size):
            configs += f"\trunner.out -> task{i}.in;\n"

        return configs
