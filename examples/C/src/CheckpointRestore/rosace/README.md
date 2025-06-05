Compile and run the `RosaceRetry.lf`


### Graph Generation.
This graph compares the code with checkpoint and restores vs with no retries, just failures.

1. Running the `RosaceRetry.lf` will create `altitudeRetry.data` and `airspeedRetry.data`.

2. Copy the code of the `RosaceController.lf` to `examples/C/src/rosace/RosaceController.lf`. This will add failures to the task, defined as `FAILURE_RATE`.

3. Run `examples/C/src/rosace/Rosace.lf`, and move the `airspeed.data` and `altitude.data` to this directory.

4. Run `python3 graph.py`.
