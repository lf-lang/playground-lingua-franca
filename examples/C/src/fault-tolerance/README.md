# Fault Tolerant Real-Time Software Examples

This PR introduces a deterministic execution model of fault-tolerant real-time software in Lingua Franca.

## Motivation
For fault tolerance, time redundancy techniques such as re-execution is widely used. 
However, this approach introduces non-determinism due to unpredictable timing of failures, which leads to inconsistent behavior and makes analysis and repeatable testing unreliable.

This PR introduces a deterministic execution model for fault-tolerant real-time software within LF.
The key idea is to re-execute failed tasks, while advancing logical execution time (LET) deterministically, regardless of actual failure timing.
This ensures that the same sequence of task failures always results in the same outcome.

We achieve this by advancing logical time using worst-case execution times (WCETs).
Furthermore, we propose an enhanced LET advancement mechanism that distinguishes between successful and failed segments.
## Key Features
### Checkpoint-Based Reexecution
- Each task is split into multiple segments and saves a checkpoint while execution.
- When the execution fails, it resumes from the last failed segment (checkpoint) on reexecution.

### Logical Execution Time (LET) Advancement
- Advances logical time by the **wcet_s** for successful segments.
- If a segment fails, LET is advanced by the **wcet_f** of the failed segment.

### Proactive Task Instance Abortion
- Before re-executing a failed segment, the system predicts whether the task can finish within its deadline, based on the logical time.
- If the prediction exceeds the deadline, the task is dropped.

## Examples
### `OneTask.lf` - Execution of a single Task
- This example runs a single task, performing checkpointed re-executions, LET advancement, and proactive abortion.
- The `TaskScheduler` periodically sends a signal to start a task, the `Task` executes, and if the Task fails, the `Coordinator` determines whether to re-execute the task, depending on the deadline, and signals to re-execute from which segment to start.

<img width="855" alt="image" src="img/OneTask.png" />

---

### `ParallelTask.lf` -  Parallel Execution of Tasks
- Adds a second `Task` instance that runs **concurrently** with the first.
- Both tasks perform checkpointed re-executions, LET advancement, and proactive abortion independently.
- It has different number of segments, WCET, optWCET.
<img width="742" alt="image" src="img/ParallelTask.png" />

---
### SequentialTask.lf` -  Sequential Execution of Tasks
- Adds a second `Task` that runs **after** the first task finishes.
- The first task's output is passed to the second task's output. 
<img width="1127" alt="image" src="img/SequentialTask.png" />

---

## ROSACE implementation
To show that this template can be used in real tasks, we implement the ROSACE benchmark.
There is already a [current ROSACE implementation](https://github.com/lf-lang/playground-lingua-franca/tree/main/examples/C/src/rosace).
We add two more versions based on the current implementation:
- `RosaceFailureWithNoReexecution.lf` : Adding random failures.
- `RosaceFailureWithReexecution.lf` : Adding random failures, and re-executing from checkpoints when failed. 
To show that our template can be easily used by using the exact C code as a task, we bring the original C implementation from the [original code](https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/).


# Implementation Details

## Key Reactors.
<img width="1127" alt="image" src="img/TaskExample.png" />

- `Trigger` :
This triggers the segment to execute. If the `instantiate` port is triggered, execution starts from the first segment. If the `retry` port is triggered, it resumes from the indicated segment number passed by the port.

- `TaskBody` :
This reactor contains user-defined C code for the task inside the preamble. Users can define task-specific inputs and outputs here.

- `AdvanceTime` :
This computes the logical execution time and advances the logical time.

- `TaskOutHandler` (optional) :
This can be added when the task output exists. This ensures that the task output is emitted only when the task succeeds.

- `Coordinator` (not part of the `Task`) :
The `Coordinator` is not part of the `Task`. This reactor manages the proactive task abortion. It receives the segment number to retry from the Task reactor, and determines whether the remaining segments can still meet the deadline. If not, it halts execution early, otherwise, it triggers a retry through the Task reactor.
On startup, the `Task` reactor sends its configuration as a struct of `task_info_t` to the corresponding `Coordinator` reactor. This includes the `task_num`, `dead_line`, `num_of_segs`, `wcet_f`, and `wcet_s`. This is for the modular design to separate the `Task` and the `Coordinator`, and not duplicate the input parameters of the `Task`.

## Usage
To use this, the user needs to `import FaultTolerantTaskTemplate, Coordinator from "lib/FaultTolerantTaskTemplate.lf"`.
The user then creates a `TaskBody`, and adds their task inside the `preamble`.
The user finally creates a reactor `extend`ing the `FaultTolerantTaskTemplate` reactor, and instantiates the `TaskBody`.
Finally, the user should configure the parameters when instantiating the `new Task()`.

### Task Parameters
- The `Task` reactor accepts the following parameters:
  - `task_num`: Task identifier.
  - `dead_line`: Logical time deadline. Different from the LF's deadline as a lag.
  - `num_of_segs`: Number of task segments.
  - `wcet_f`: Array of worst-case execution times of segments in msecs.
  - `wcet_s`: Array of optimized execution times of segments in msecs.
- The user can additionally pass the task's input in the `extend`ing the `TaskBody`, and also set task outputs creating a `TaskOutHandler`.

This is an example how it looks.
```
  task = new Task1<int, int, int>(
      task_num=1,
      dead_line=3000,
      num_of_segs=3,
      wcet_f = {300, 300, 300},
      wcet_s = {250, 250, 250})
```

## Evaluation
There is a directory for evaluation, inside `evaluation`. The `evaluation.sh` will create `.lf` files for evaluation, compile and execute, and log the results.
It compares three cases, 
- Baseline: No LET advance, only re-executing until deadline misses.
- Proposed1: Advance LET as much as `wcet_f` even if the segment succeeds, and do proactive task abortion.
- Proposed2: Advance LET as much as `wcet_f`when segment fails, and `wcet_s` when segment succeeds, and do proactive task abortion.

We compare the Deadline misses, Execution failures, and the Sum of deadline misses and execution failures, and CPU utilization.

Graph generation is also possible in the `evaluation/graph_generation` directory.

The Rosace implementation also includes a directory named `rosace/graph_generation`.
It compares the results with and without re-execution based fault-tolerance.
