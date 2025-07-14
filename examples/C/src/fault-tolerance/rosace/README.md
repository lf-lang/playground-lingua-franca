Compile and run the `RosaceRetry.lf`


### Graph Generation.
This graph compares the code with checkpoint and restores vs with no retries, just failures.

1. Running the `RosaceRetry.lf` will create `altitudeRetry.data` and `airspeedRetry.data`.

2. Copy the code of the `RosaceController.lf` to `examples/C/src/rosace/RosaceController.lf`. This will add failures to the task, defined as `FAILURE_RATE`.

3. Run `examples/C/src/rosace/Rosace.lf`, and move the `airspeed.data` and `altitude.data` to this directory.

4. Run `python3 graph.py`.
We execute the ROSACE benchmark including failures and using retries.

TODO: /////////////////

### Only adding failures.
`RosaceFailureWithNoRetry.lf` is same as the example in `examples/C/src/rosace/Rosace.lf`, however, it uses a faulty controller `RosaceControllerWithFailure.lf`.
`RosaceControllerWithFailure.lf` adds a random failure, with a configurable fault rate.

### Adding failures and checkpointed retries.
`RosaceFailureWithRetry.lf` adds failures, and checkpointed retries. The controller is imported from `RosaceControllerWithRetry.lf`.
We try to use the original C code from the rosace benchmark, such as the `Va_control_50()` function.


### Note
`build_run_plot.sh` and `rosace.gnuplot` is same as in `examples/C/src/rosace/`, however copied here due to path problems when executing.
