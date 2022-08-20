# Basic Plot
# x axis: number of workers
# y axis: physical execution time

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import matplotlib
matplotlib.use('tkagg')

import os
import sys
import subprocess
import shutil
from functools import partial
import statistics

class PlotGenerator(object):
    
    def __init__(self):
        self.config = {
            'title': '',
            'x-axis': 'num_worker',
            'y-axis': 'physical_excution_time',
            'dataset': {},
            'num_iteration': 1
        }

    def setConfig(self, config):

        for key, value in config.items():
            if key in self.config.keys():
                self.config[key] = value

    def plot_graph(self):
        
        LF_PATH = os.getenv("LF_PATH")
        if LF_PATH == None:
            sys.exit("Set the environment variable LF_PATH to the path where Lingua Franca is installed")
    
        os.chdir(LF_PATH)
        workers = self.config['dataset']['workers']

        target_schedulers = []
        for scheduler in self.config['dataset']['schedulers'].keys():
            if len(self.config['dataset']['schedulers'][scheduler]) > 0:
                target_schedulers.append(scheduler)

        if len(target_schedulers) == 0:
            print("There is no LF file to plot")
            return

        exe_times = {}
        for scheduler in target_schedulers:
            exe_time = []

            for i, worker in enumerate(workers):
            
                exe_time.append(statistics.mean([self.__run_single_LF(self.config['dataset']['schedulers'][scheduler][i]) for _ in range(int(self.config['num_iteration']))]))
            exe_times[scheduler] = exe_time.copy()
            exe_time.clear()
        

        fig, ax = plt.subplots()
        plt.axis([1, 25, 0.0, 5.0])        
        colors = ['#D81B60', '#1E88E5', '#FFC107', '#004D40', '#8794DD']
        patches = []
        axes = []
        for i, scheduler in enumerate(target_schedulers):
           patches.append(mpatches.Patch(color=colors[i], label=scheduler))
           axes.append(ax.plot(workers, exe_times[scheduler], '--', color=colors[i]))

        ax.legend(handles=patches, loc='upper right')
        plt.axis([0, max(workers)+1, 0, max(max(exe_times[s]) for s in target_schedulers) * 1.2])
        
        plt.xlabel('Number of Worker')
        plt.ylabel('Physical Execution time')

        plt.title(self.config['title'], fontsize= 10)
        plt.show()


    def __run_single_LF(self, filepath):

        LF_PATH = os.getenv("LF_PATH")
        if LF_PATH == None:
            sys.exit("Set the environment variable LF_PATH to the path where Lingua Franca is installed")
    
        os.chdir(LF_PATH)

        if os.path.isfile(filepath) == False:
            sys.exit("The LF file is not exists.")
        
        filename = filename = filepath.split('/')[-1].split('.')[0]
        binpath = f"{'/'.join(filepath.split('/')[:-3])}/bin/{filename}"

        out = subprocess.run([f'./gradlew', 'runLfc', '--args', filepath], capture_output=True)
        built_success = False
        for line in reversed(out.stdout.decode("utf-8").split("\n")):
            if line.startswith("BUILD SUCCESSFUL"):
                built_success = True
                break
        
        if built_success:
            print(f"Built Successfull: {filepath}")
        else:
            print("Build Failed")
            print(out.stderr.decode("utf-8"))
            exit(0)
    
        lf_out = subprocess.run([binpath], capture_output=True)

        for line in reversed(lf_out.stdout.decode("utf-8").split("\n")):
            if line.startswith("---- Elapsed physical"):
                exe_time = line.split(' ')[-1]
                break

        exe_time = int(exe_time.replace(',','')) / 1000000000
        return exe_time
