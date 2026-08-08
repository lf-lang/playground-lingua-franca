We execute the ROSACE benchmark including failures and using re-executions.

### Adding failures and checkpointed re-executions.
`RosaceWithReExecution.lf` adds failures, and checkpointed re-executions. The controller is imported from `RosaceControllerWithReExecution.lf`.
We try to use the original C code implementation from the rosace benchmark, such as the `Va_control_50()` function.
Another version only adding random failures without reexecution is implemented [here](https://github.com/asu-kim/fault-tolerant-real-time/blob/main/fault-tolerance/rosace/RosaceFailureWithNoReexecution.lf).
