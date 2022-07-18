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
            'utilization': 0.6
        }

        if TEMPLATE_PATH == '':
            self.template = self.make_template()
        else:
            if os.path.isfile(TEMPLATE_PATH) == True:
                with open(TEMPLATE_PATH) as f:
                    self.template = f.read()

    def setConfig(self, config):
        for key, value in config.items():
            if key in self.config.keys():
                self.config[key] = value


    def makeLF(self, saveDir='./'):
        if os.path.isdir(saveDir) == False:
            sys.exit("The directory for saving result is not exist.")
        
        char_to_replace = {}
        char_to_replace['$TOTAL_TIME$'] = f'{self.config["timeout"]["value"]} {self.config["timeout"]["timeUnit"]}'
        char_to_replace['$UTILIZATION$'] = str(self.config['utilization'])
        char_to_replace['$PERIODIC$'] = 'true' if self.config['periodicity'] == 'periodic' else 'false'
        char_to_replace['$PERIOD$'] = f'{self.config["period"]["value"]} {self.config["period"]["timeUnit"]}'
        char_to_replace['$NUM_TASKS$'] = str(self.config['num_tasks'])
        
        workers = [w for w in range(self.config['min_workers'], self.config['max_workers']+1)]
        generated_files = []
        
        for scheduler in self.config['schedulers']:
            char_to_replace['$SCHEDULER_TYPE$'] = scheduler
            for worker in workers:
                char_to_replace['$NUM_WORKERS$'] = str(worker)
                FILE_NAME = f'{self.config["periodicity"].capitalize()}_{scheduler}_{worker}.lf'
                FILE_PATH = f'{saveDir}/{FILE_NAME}'

                contents = self.template
                for k, v in char_to_replace.items():
                    contents = contents.replace(k, v)
                
                with open(FILE_PATH, 'w') as lf_file:
                    lf_file.write(contents)
                    lf_file.close()
                
                if os.path.isfile(FILE_PATH) == True:
                    print(f'File saved: {FILE_PATH}')
                    generated_files.append(FILE_PATH)
                else:
                    sys.exit('The LF file is not generated.')
        
        return generated_files                

    
    def make_template(self):
        template = """/**
 * Taskset Generator of Lingua Franca
 * @author Hokeun Kim
 * @author Yunsang Cho
 */

target C {
    timeout: $TOTAL_TIME$,
    workers: $NUM_WORKERS$,
    scheduler: $SCHEDULER_TYPE$
};

preamble {=
    typedef struct task_config_t {
        int id;
        long long int total_time;
        long long int release_time;
        long long int exe_time;
        bool    periodic;
        long long int period;
    } task_config_t;
    //#define 
=}

// Each Task has information about its release time and exeuction time
reactor Task {

    input in:task_config_t;
    state total_time:time(0 sec);
    state exe_time:time(0 sec);
    state id:int;
    state periodic:bool(false);
    state period:time(0 sec);
    logical action release;

    reaction(startup) {=
        #ifdef TASK_SET_TRACING_ON
            if (!register_user_trace_event("ID")) {
                fprintf(stderr, "ERROR: Failed to register trace event.");
                exit(1);
            }
        #endif // TASK_SET_TRACING_ON
    =}

    reaction(in) -> release {=
        self->id = in->value.id;
        self->total_time = in->value.total_time;
        self->exe_time = in->value.exe_time;
        self->periodic = in->value.periodic;
        self->period = in->value.period;
        lf_schedule(release, in->value.release_time);
    =}
    
    reaction(release) -> release {=
        long long int physical_start_time = lf_time_physical();
        //tracepoint_user_value("ID", self->id);
        printf("Task %d released at logical time %lld nsec, physical time %lld nsec, execution time %lld nsec",
            self->id,
            lf_time_logical_elapsed(),
            lf_time_physical_elapsed(),
            self->exe_time
        );
        while (lf_time_physical() < physical_start_time + self->exe_time) {

        };
        printf("Task %d finished execution at physical time %lld nsec",
            self->id,
            lf_time_physical_elapsed());
        if (self->periodic) {   // periodic task
            lf_schedule(release, self->period);
        } else {            // sporadic task
            srand(0);
            float r = (float) rand() / (float) RAND_MAX;
            long long int period = self->exe_time + (long long int) (r * self->total_time);
            lf_schedule(release, period);
        }
    =}
}

// Runner with uniform task execution time and release time = 0.
reactor SimpleRunner(num_task:int(10), total_time:time(10 sec), utilization:float(0.6), periodic:bool(false), period:time(1 sec)) {
    output[100] out:task_config_t;
    
    reaction(startup) -> out {=
        long long int exe_time;
        if (self->periodic) {
            exe_time = (long long int) ((self->utilization * self->period) / self->num_task);
        } else {
            exe_time = (long long int) ((self->utilization * self->total_time) / (2 * (self->num_task - self->utilization)));
        }
        for (int i = 0; i < self->num_task; i++) {
            task_config_t message;
            message.id = i;
            message.total_time = self->total_time;
            message.exe_time = exe_time;
            message.release_time = (long long int) 0;
            message.periodic = self->periodic;
            message.period = (long long int) self->period;
            lf_set(out[i], message);
        }
    =}
}

// Runner considering number of workers when computing execution time with uniform task execution time and release time = 0.
reactor RunnerForMultipleWorkers(num_task:int(100), total_time:time(10 sec), utilization:float(0.6)) {
    output[100] out:task_config_t;

    reaction(startup) -> out {=
        long long int exe_time = (long long int) $NUM_WORKERS$ * ((self->total_time * self->utilization) / self->num_task);
        for (int i = 0; i < self->num_task; i++) {
            task_config_t message;
            message.id = i;
            message.exe_time = exe_time;
            message.release_time = (long long int) 0;
            lf_set(out[i], message);
        }
    =}
}

// Runner which set tasks' release times with random number.
reactor RandomReleaseRunner(num_task:int(100), total_time:time(10 sec), utilization:float(0.6)) {
    output[100] out:task_config_t;

    reaction(startup) -> out {=
        srand(time(0));
        long long int exe_time = (long long int) $NUM_WORKERS$ * ((self->total_time * self->utilization) / self->num_task);
        for (int i = 0; i < self->num_task; i++) {
            task_config_t message;
            float r = (float) rand() / (float) RAND_MAX;
            message.id = i;
            message.exe_time = exe_time;
            message.release_time = (long long int) (r * (float) (self->total_time - exe_time) / 10.0);
            lf_set(out[i], message);
        }
    =}
}

main reactor {
    runner = new SimpleRunner(num_task=$NUM_TASKS$, total_time=$TOTAL_TIME$, utilization=$UTILIZATION$, periodic=$PERIODIC$, period=$PERIOD$);
    //runner = new RunnerForMultipleWorkers();
    //runner = new RandomReleaseRunner();
    tasks = new[100] Task();
    runner.out -> tasks.in;
}
        """
        return template

