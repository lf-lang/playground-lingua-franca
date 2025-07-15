### Graph Generation.
This graph compares the code with checkpoint and restores vs with no retries, just failures.

1. Running the `RosaceFailureWithRetry.lf` will create `altitudeRetry.data` and `airspeedRetry.data`.

2. Change `altitude.data` and `airspeed.data` to `altitudeRetry.data` and `airspeedRetry.data`.

3. Run `RosaceFailureWithNoRetry.lf`.

4. Run `python3 graph.py`.