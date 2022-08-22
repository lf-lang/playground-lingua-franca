# Taskset Generator's Commend Line Interface
#
# 
# Required 
# # Set schdeulers
# 1. "-S", "--schedulers": string type
#    -> "Choose the schedulers: 'NP', 'GEDF_NP', 'GEDF_NP_CI, 'adaptive'"
#
# # Set General Configuration
# 2. "-NI", "--num_iteration": int type
#    -> "Set the number of iterations"
# 3. "-D", "--deadline": string type
#    -> "Set the dealine(ex. 1 sec); choose the unit(sec, msec, usec, nsec)"
# 4. "-TT", "--total_time": string type
#    -> "Set the total time(ex. 1 sec); choose the unit(sec, msec, usec, nsec)"
#
# # Choose the type of task
# 5. "-T", "--type": string type
#    -> "Choose the type of taskset: 'basic', 'dag'"
#
# Optional
# # Basic Taskset
# 1. "-P", "--periodicity": string type
#    -> "Choose the peridicity: 'sporadic', 'periodic'"
# 2. "-NT", "--num_tasks": int type
#    -> Set the number of tasks"
# 3. "-U", "--utilization": float type
#    -> "Set the utilization"
# 4. "--period": string type
#    -> "Set the period(ex. 1 sec); choose the unit(sec, msec, usec, nsec)"
#
# # DAG Taskset
# 1. "-NL", "--num_level": int type
#    -> "Choose the number of level"
# 2. "-MNC", "--max_num_components": int type
#    -> "Set the maximum number of components in one level"
# 3. "-TC", "--components_time": string type
#    -> "Set the exectuion time of each component(ex. 1 sec); choose the unit(sec, msec, usec, nsec)"

# # Optional setting random seed
# 0. "--seed": int type
#    -> "Can set the random seed if the type is 'dag' or 'sporadic basic'"

python3 cli.py -S NP -NI 1 -D 1 sec -TT 1 sec -T basic -P sporadic -NT 1 -U 0.4 --period 1 sec
