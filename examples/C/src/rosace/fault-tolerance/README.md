We execute the ROSACE benchmark including failures and using re-executions.

### Only adding failures.
`RosaceFailureWithNoReexecution.lf` is same as the example in `examples/C/src/rosace/Rosace.lf`, however, it uses a faulty controller `RosaceControllerWithFailure.lf`.
`RosaceControllerWithFailure.lf` adds a random failure, with a configurable fault rate.

### Adding failures and checkpointed re-executions.
`RosaceFailureWithReexecution.lf` adds failures, and checkpointed re-executions. The controller is imported from `RosaceControllerWithReexecution.lf`.
We try to use the original C code implementation from the rosace benchmark, such as the `Va_control_50()` function.

### Note
`build_run_plot.sh` and `rosace.gnuplot` is same as in `examples/C/src/rosace/`, however copied here due to path problems when executing.

### Graph generation
The `graph_generation` creates a graph comparing the results.