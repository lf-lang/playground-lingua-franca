### Graph Generation.
This graph compares using checkpoint and restores, to no re-executions only including failures.

1. Running the `RosaceFailureWithReexecution.lf` will create `altitude.data` and `airspeed.data`.

2. Change `altitude.data` and `airspeed.data` to `altitudeRe-execution.data` and `airspeedRe-execution.data`.

3. Run `RosaceFailureWithNoReexecution.lf`.

4. Run `python3 graph.py`.