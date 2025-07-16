### Graph Generation.
This graph compares using checkpoint and restores, to no retries only including failures.

1. Running the `RosaceFailureWithRetry.lf` will create `altitudeRetry.data` and `airspeedRetry.data`.

2. Change `altitude.data` and `airspeed.data` to `altitudeRetry.data` and `airspeedRetry.data`.

3. Run `RosaceFailureWithNoRetry.lf`.

4. Run `python3 graph.py`.