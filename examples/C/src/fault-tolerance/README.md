# Fault Tolerant Real-Time Software Examples

The following examples demonstrate modeling and simulation of fault-tolerant real-time software tasks in Lingua Franca.

## CheckpointRestore/RetryOneTask.lf

### Checkpoint-Based Retry
- Each task is split into multiple segments, and execution resumes from the last failed segment (checkpoint) on retry.

### Logical Elapsed Time (LET) Advancement
- Advances logical time by the **optimized execution time (optWCET)** for successful segments.
- If a segment fails, LET is advanced by the **worst-case execution time (WCET)** of the failed segment.

### Predictive Task Dropping
- Before retrying a failed segment, the system predicts whether the task can finish within its deadline.
- If the prediction exceeds the deadline, the task is dropped.
<img width="855" alt="image" src="https://github.com/user-attachments/assets/1f6b8ca2-6969-44a5-8d81-1eb3b34755ee" />


---

## CheckpointRestore/RetryParallelTask.lf

### Parallel Execution of Tasks
- Adds a second `Task` instance that runs **concurrently** with the first.
- Both tasks perform checkpointed retries, LET advancement, and predictive dropping independently.
- It has different number of segments, WCET, optWCET.
<img width="742" alt="image" src="https://github.com/user-attachments/assets/58604432-2f76-4976-acbf-4d38d8bcf007" />

---
## CheckpointRestore/RetrySequentialTask.lf
### Sequential Execution of Tasks
- Adds a second `Task` that runs **after** the first task finishes.
- The deadline of the second task is computed from the _start of the second task_.
<img width="1127" alt="image" src="https://github.com/user-attachments/assets/5523546f-a570-4923-b9f4-d239c0220152" />
---
## Implementation Details

### Modular design
The user needs to `import TaskTemplate, CheckDeadlineAndRetry, TaskScheduler from "lib/RetryTask.lf"`.
The user then creates a reactor `extend`ing the `TaskTemplate` reactor, and add their task inside the preamble.
Finally, the user should configure the parameters when instantiating the `new Task()`.

### Task Parameters
- The `Task` reactor accepts the following parameters:
  - `task_num`: Task identifier.
  - `dead_line`: Logical time deadline. Different from the LF's deadline as a lag.
  - `num_of_segs`: Number of task segments.
  - `wcet`: Array of worst-case execution times in msecs.
  - `optwcet`: Array of optimized execution times in msecs.

### Parameter Sending
- On startup, the `Task` reactor sends its configuration to the corresponding `CheckDeadlineAndRetry` reactor.

## Limitation
One limitation is that LF does not support (or maybe cannot) passing function pointers as a `Reactor` input parameter.
The current implementation of this task just uses the `task_number` to link a predefined empty function `total_task'n'`, to the user's implementation.

```
// RetryTask.lf
  preamble {=
    int total_task1(int checkpoint);
    int total_task2(int checkpoint);
  =}

  method set_fn_ptr() {=
    if(self->task_num == 1) {
      self->task_fn = total_task1;
    } else if(self->task_num == 2) {
    self->task_fn = total_task2;
    }
  =}
```

Without this, we cannot use different tasks (with different implementations), or else it will give a duplicate definition error on the task.