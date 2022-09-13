import os
from datetime import datetime 
from tasksets.BasicTaskSet import BasicTaskSet
from tasksets.DagTaskSet import DagTaskSet
from tasksets.CustomTaskSet import CustomTaskSet


class TasksetGenerator(object):
    
    def __init__(self):
        self.config = {
            'schedulers': ['NP'],
            'type': 'basic',
            'timeout': {'value': 10, 'timeUnit': 'sec'},
            'min_workers': 1,
            'max_workers': 20,
            'deadline': {'value': 100, 'timeUnit': 'msec'}
        }
        self.basic_config = {
            'periodicity': 'sporadic',
            'period': {'value': 0, 'timeUnit': 'sec'},
            'num_tasks': 20,
            'utilization': 0.6,
            'seed': datetime.now()
        }
        self.dag_config = {
            'seed': int(round(datetime.now().timestamp())),
            'max_depth': 5,
            'num_outputs': 4,
            'execution_time': {'value': 100, 'timeUnit': 'msec'}
        }

    def setConfig(self, config) :
        configs = [self.config, self.basic_config, self.dag_config]

        for key, value in config.items():
            for c in configs:
                if key in c.keys():
                    c[key] = value
    
    # Generate taskset LF files to speicific directory.
    def makeLF(self, templateDir='./', outputDir='./', template_path=''):
        TEMPLATE_PATH = f'{templateDir}/{self.config["type"].capitalize()}TaskSetGeneratorTemplate.lf'
        if len(template_path) > 0:
            TEMPLATE_PATH = template_path

        if not os.path.isfile(TEMPLATE_PATH):
            raise RuntimeError("No template file of task generator: " + TEMPLATE_PATH)
        if not os.path.exists(outputDir):
            # Create output directory if not exists.
            os.makedirs(outputDir)
            if not os.path.isdir(outputDir):
                raise RuntimeError("No output directory: " + outputDir)
        

        char_to_replace = {
            '$TOTAL_TIME$': f'{self.config["timeout"]["value"]} {self.config["timeout"]["timeUnit"]}'
        }   

        if self.config['type'] == 'basic':
            basic_taskset = BasicTaskSet(TEMPLATE_PATH=TEMPLATE_PATH)
            basic_taskset.setConfig(self.config)
            basic_taskset.setConfig(self.basic_config)
            generated_files = basic_taskset.makeLF(outputDir=outputDir)

        elif self.config['type'] == 'dag':
            dag_taskset = DagTaskSet(TEMPLATE_PATH=TEMPLATE_PATH)
            dag_taskset.setConfig(self.config)
            dag_taskset.setConfig(self.dag_config)
            generated_files = dag_taskset.makeLF(outputDir=outputDir)
        
        elif self.config['type'] == 'custom':
            custom_taskset = CustomTaskSet(TEMPLATE_PATH=TEMPLATE_PATH)
            custom_taskset.setConfig(self.config)
            generated_files = custom_taskset.makeLF(outputDir=outputDir)
        

        return generated_files
