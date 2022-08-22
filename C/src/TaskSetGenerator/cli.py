# A GUI program for task set generator. (Command line interface version.)
# @author Wonseo Choi

from plots import SavePlot

import os
from datetime import datetime

import argparse
import csv

from TasksetGenerator import TasksetGenerator

class CLI(object):
    def __init__(self):
        self.taskConfig = {
            'schedulers':[]
        }

        parser = argparse.ArgumentParser(description="A GUI program for task set generator. (Command line interface version.)")
        # Set schdeulers
        parser.add_argument("-S", "--schedulers", type=str, required=True,
                            help="Choose the schedulers: 'NP', 'GEDF_NP', 'GEDF_NP_CI, 'adaptive'")
        # Set General Configuration
        parser.add_argument("-NI", "--num_iteration", type=int, required=True,
                            help="Set the number of iterations")
        parser.add_argument("-D", "--deadline", type=str, required=True, default="1_sec",
                            help="Set the dealine(ex. 1sec); can choose the unit(sec, msec, usec, nsec)")
        parser.add_argument("-TT", "--total_time", type=str, default="1_sec",
                            help="Set the total time(ex. 1sec); can choose the unit(sec, msec, usec, nsec)")
        
        # Choose the type of task
        parser.add_argument("-T", "--type", type=str, required=True,
                            help="Choose the type of taskset: 'basic', 'dag'")
       
        # Basic Taskset
        parser.add_argument("-P", "--periodicity", type=str,
                            help="Choose the peridicity: 'sporadic', 'periodic'")
        parser.add_argument("-NT", "--num_tasks", type=int,
                            help="Set the number of tasks")
        parser.add_argument("-U", "--utilization", type=float,
                            help="Set the utilization")
        parser.add_argument("--period", type=str, default="1_sec",
                            help="Set the period(ex. 1sec); can choose the unit(sec, msec, usec, nsec)")
        
        # DAG Taskset
        parser.add_argument("-NL", "--num_level", type=int,
                            help="Choose the number of level")
        parser.add_argument("-MNC", "--max_num_components", type=int,
                            help="Set the maximum number of components in one level")
        parser.add_argument("-TC", "--components_time", type=str, default="1_sec",
                            help="Set the exectuion time of each component(ex. 1sec); can choose the unit(sec, msec, usec, nsec)")
        
        # Optional setting random seed
        parser.add_argument("--seed", type=int, default=1234,
                            help="Can set the random seed if the type is 'dag' or 'sporadic basic'")

        self.args, _ = parser.parse_known_args()

    def Run(self):
        self.setConfig()
        WORKING_DIR = os.getcwd()

        generator = TasksetGenerator()
        generator.setConfig(self.taskConfig)
        generated_files = generator.makeLF(templateDir=f'{WORKING_DIR}/templates', outputDir=f'{WORKING_DIR}/.gui/src/')
        print("Finished generating LF files!")
        
        plot_title = ''
        if self.taskConfig['type'] == 'basic':
            plot_title = f'{self.taskConfig["periodicity"].capitalize()} / Number of task: {self.taskConfig["num_tasks"]} / Utilization: {self.taskConfig["utilization"]}'
            save_name = f'basic/{self.taskConfig["periodicity"]}-num_tasks_{self.taskConfig["num_tasks"]}-util_{self.taskConfig["utilization"]}'
        elif self.taskConfig['type'] == 'dag':
            plot_title = f'DAG / Seed: {self.taskConfig["seed"]}'
            save_name = f'dag/seed_{self.taskConfig["seed"]}'
            
        
        plot_generator = SavePlot.PlotGenerator()
        plot_generator.setConfig({
            'title': plot_title,
            'dataset': generated_files,
            'num_iteration': self.args.num_iteration,
            'save_name': save_name
        })

        WORKING_DIR = os.getcwd()
        output_dir = f'{WORKING_DIR}/.output'
        if not os.path.exists(output_dir):
            os.mkdir(output_dir)
        
        output_dir = f'{output_dir}/{int(round(datetime.now().timestamp()))}'
        if not os.path.exists(output_dir):
            os.mkdir(output_dir)
            os.mkdir(os.path.join(output_dir, self.taskConfig['type']))
        
        exe_times, deadline_misses = plot_generator.plot_graph(output_dir)
        result = {
            'workers': [i for i in range(1, 21)],
            'exe_times': exe_times,
            'deadline_misses': deadline_misses
        }

        self.saveResult(result, output_dir)

    def setConfig(self):

        # self.taskConfig['schedulers'] = self.args.schedulers
        self.taskConfig['schedulers'] = ["NP"]
        
            
        self.taskConfig['type'] = self.args.type

        self.taskConfig['timeout'] = {
            'value': int(self.args.total_time.split("_")[0]),
            'timeUnit': self.args.total_time.split("_")[1]
        }

        # FIXME: Should add spinboxs for min_workers and max_workers in GUI.
        self.taskConfig['min_workers'] = 1
        self.taskConfig['max_workers'] = 20

        self.taskConfig['deadline'] = {
            'value': int(self.args.deadline.split("_")[0]),
            'timeUnit': self.args.deadline.split("_")[1]
        }

        if self.taskConfig['type'] == 'basic':
            self.taskConfig['periodicity'] = self.args.periodicity
            self.taskConfig['period'] = {
               'value': int(self.args.period.split("_")[0]),
               'timeUnit': self.args.period.split("_")[1]
            }
            self.taskConfig['num_tasks'] = self.args.num_tasks
            self.taskConfig['utilization'] = self.args.utilization
            self.taskConfig['seed'] = self.args.seed if self.args.seed else int(round(datetime.now().timestamp()))
        
        elif self.taskConfig['type'] == 'dag':
            self.taskConfig['seed'] = self.args.seed if self.args.seed else int(round(datetime.now().timestamp()))
            self.taskConfig['max_depth'] = self.args.max_depth
            self.taskConfig['num_outputs'] = self.args.num_outputs
            self.taskConfig['execution_time'] = {
               'value': int(self.args.execution_time.split("_")[0]),
               'timeUnit': self.args.execution_time.split("_")[1]
            }

    def saveResult(self, result, output_dir):
        
        if self.taskConfig['type'] == 'basic':
            header = ['type', 'number of iterations', 'deadline', 'schedulers', 'number of tasks', 'total time', 'utilization', 'periodicity']
            configs = ['basic', self.spinBox_numOfIterations.value(), f'{self.taskConfig["deadline"]["value"]} {self.taskConfig["deadline"]["timeUnit"]}', 
                        ', '.join(self.taskConfig['schedulers']), self.taskConfig['num_tasks'], f'{self.taskConfig["timeout"]["value"]} {self.taskConfig["timeout"]["timeUnit"]}',
                        self.taskConfig['utilization'], self.taskConfig['periodicity']
                      ]
            if self.taskConfig['periodicity'] == 'sporadic':
                header.append('seed')
                configs.append(self.taskConfig['seed'])
            elif self.taskConfig['periodicity'] == 'periodic':
                header.append('period')
                configs.append(f'{self.taskConfig["period"]["value"]} {self.taskConfig["period"]["timeUnit"]}')

        elif self.taskConfig['type'] == 'dag':
            header = ['type', 'number of iterations', 'deadline', 'schedulers', 'number of level', 'maximum components in each level', 'seed', 'execution time']
            configs = ['dag', self.spinBox_numOfIterations.value(), f'{self.taskConfig["deadline"]["value"]} {self.taskConfig["deadline"]["timeUnit"]}', 
                        ', '.join(self.taskConfig['schedulers']), self.taskConfig['max_depth'], self.taskConfig['num_outputs'], self.taskConfig['seed'],
                        f'{self.taskConfig["execution_time"]["value"]} {self.taskConfig["execution_time"]["timeUnit"]}'
                      ]

        output_file = f'{output_dir}/{self.taskConfig["type"]}-{", ".join(self.taskConfig["schedulers"])}.csv'
        
        with open(output_file, 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)

            writer.writerow(header)
            writer.writerow(configs)


            outputs_header = ['scheduler', 'worker', 'physical execution time', 'deadline miss']
            outputs = []

            for scheduler in self.taskConfig['schedulers']:
                for i, worker in enumerate(result['workers']):
                    output = [scheduler, worker, result['exe_times'][scheduler][i], result['deadline_misses'][scheduler][i]]
                    outputs.append(output.copy())
                    output.clear()
            
            writer.writerow(outputs_header)
            writer.writerows(outputs)


if __name__ == "__main__":
    cli = CLI()
    cli.Run()
