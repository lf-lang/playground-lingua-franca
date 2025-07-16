This directory includes libraries for evaluation.

- `TaskRetryTemplate.lf`: 
Additionally includes `TaskRetryWorstCaseTemplate` and `AdvanceTimeWorstCase`, which always advances as much as the `wcet_f`, even when segment succeeds.
- `TaskRetryNoAdvanceTemplate.lf`:
Additionally includes `TaskRetryNoAdvance` and `NoTaskDrop`, which does not advance logical time, and does not compute the time left until the deadline, and of course does not abort task depending on the deadline.
- `fault_tolerance_common.h`:
Library function for the task segments.
