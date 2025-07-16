### Evaulation
This creates evaluation results with configurable failure rates and number of executions.

1. Make sure lfc is in PATH, and install gtime or time depending on OS.
2. Fix the configurations in `eval.sh` top, (NUM, failure_rates)
3. Run `eval.sh` in current directory.


### Graph generation.
Check graph_generation/README.md for graph generation.

1. `python3 gen_csv.py` in current directory.
2. `python3 graph_combined.py` in current directory.
