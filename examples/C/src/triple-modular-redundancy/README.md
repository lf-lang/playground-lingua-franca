# Triple Modular Redundancy in Time-Sensitive Systems

## Overview

This playground example demonstrates the scheduling of the triple modular redundancy (TMR) mechanism in time-sensitive systems.

# Deadline-Bounded Re-execution for Soft-Error Resilience using Reactor Models and Lingua Franca (Currently under revision for TCRS 2026 / IEEE Embedded Systems Letters.)

## Description

This example provides a high-level, abstracted failure simulation on Lingua Franca (LF) to simulate the effect of re-execution for fault-aware MCSs under potential DUEs and SDCs.

Specifically, the first example (SerialTMR_physical.lf) models the deadline-bounded re-execution strategy, which estimates the SDC probability of each completed execution based on the given fault rate, conditional failure probabilities, and the measured physical execution time. This example relies on the deadline construct of LF. If the remaining slack time is less than the WCET, the deadline handler of the re-execution decision reaction terminates further re-execution. Otherwise, the body of the re-execution decision reaction decides whether further re-execution is required based on the SDC estimation.

The second example (SerialTMR_physical_baselines.lf) implements fixed re-execution strategies based on the maximum number of total executions (N) and maximum number of completed executions (M) in [1]. By configuring parameters N and M, this baseline can simulate re-execution strategies such as [1], [2], and [3] at a high level. Note that the baseline does not include the analytical failure formulations and corresponding re-execution parameter selection; the user should manually configure failure probabilities and proper re-execution parameters (N and M).

[1] SO, Hwisoo, et al. PREFACE: Proactive Re-executions for Fault-aware Mixed-criticality Environments. In: 2026 Design, Automation & Test in Europe Conference (DATE). IEEE, 2026. p. 1-7.
[2] REGHENZANI, Federico, et al. A mixed-criticality approach to fault tolerance: Integrating schedulability and failure requirements. In: 2022 IEEE 28th Real-Time and Embedded Technology and Applications Symposium (RTAS). IEEE, 2022. p. 27-39.
[3] HUANG, Shao-Yu, et al. Rtailor: Parameterizing soft error resilience for mixed-criticality real-time systems. In: 2023 IEEE Real-Time Systems Symposium (RTSS). IEEE, 2023. p. 344-357.

###  How to Use?

You can compile `SerialTMR_physical.lf` and `SerialTMR_physical_baselines.lf` with `lfc` as follows.

First, move to the root directory of the Lingua Franca Playground repository. Then run:

```bash
lfc examples/C/src/triple-modular-redundancy/SerialTMR_physical.lf
lfc examples/C/src/triple-modular-redundancy/SerialTMR_physical_baselines.lf
```

After compilation, execute the generated program with:

```bash
FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical
N=1 M=1 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
N=2 M=1 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
N=3 M=1 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
N=3 M=3 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
```

Note: this example requires Lingua Franca version 0.12.0 or later.

### How to Configure?

#### Fault rate

Both `SerialTMR_physical.lf` and `SerialTMR_physical_baselines.lf` read the fault rate per millisecond from the `FAULT_RATE` environment variable.

For example:

```bash
FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical
FAULT_RATE=0.02 ./examples/C/bin/SerialTMR_physical
FAULT_RATE=0.002 ./examples/C/bin/SerialTMR_physical
```

If FAULT_RATE is not specified, the default value is 0.2 per millisecond.

#### Re-execution parameters for the baselines

`SerialTMR_physical_baselines.lf` additionally supports the maximum number of total executions (N) and the maximum number of completed executions (M) through environment variables.

For example:
```bash
N=1 M=1 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
N=2 M=1 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
N=3 M=1 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
N=3 M=3 FAULT_RATE=0.2 ./examples/C/bin/SerialTMR_physical_baselines
```
If N and M are not specified, both default to 1.

#### Other parameters

Other parameters, such as the conditional DUE and SDC probabilities, the required failure probability, the average execution time, the range of the execution time, the task deadline, and the deadline of the re-execution decision reaction, are currently configured in the LF source code and require recompilation after modification.

## AI Use Disclosure
We used ChatGPT to assist with parts of the implementation. Generated codes are validated and edited by the authors.




# Configurable Modeling and Simulation of Resilient Real-Time Scheduling (Presented at ReCPS 2026)

## Description

This example provides a high-level, abstracted failure simulation on Lingua Franca (LF) to simulate the effect of the re-execution for fault-aware MCSs under potential DUEs and SDCs. Specifically, this example models a naive TMR, which re-executes a task twice, i.e., one execution and two additional re-executions, and proceeds with the majority voting among completed executions. Each execution in this example is classified as benign (success), DUE, or SDC with given probabilities. These probabilities are derived from a specified fault rate and task characteristics following the failure models from PREFACE, a state-of-the-art fault-aware MCS approach. Further, this example provides N-modular redundancy (NMR), e.g., 5-Modular Redundancy, and parallel execution of redundant executions on LF.

### How to Use?

You can compile `SerialTMR.lf` with `lfc` as follows.

First, move to the root directory of the Lingua Franca Playground repository. Then run:

```bash
lfc examples/C/src/triple-modular-redundancy/SerialTMR.lf
```

After compilation, execute the generated program with:

```bash
./examples/C/bin/SerialTMR
```

Note: this example requires Lingua Franca version 0.12.0 or later.

### How to Configure?

Currently, this example does not support command-line parameters. To change the configuration, manually edit the parameters defined in `SerialTMR.lf` before compilation.

You can configure DUE and SDC probabilities per execution by modifying the following variables in preamble:

```
static const double due_per_exec = 0.3;
static const double sdc_per_exec = 0.3;
```

By default, this example simulates the naive TMR. You can simulate different levels of redundancy by modifying the following variable in SerialExecution reactor:

```
state max_number_of_executions: int = 3
```

For example, if you set max_number_of_executions as 4, this example will simulate naive quadruple modular redundancy with first execution and additional three re-executions.
	
You can change the random seed with the following variable in preamble:

```
static uint64_t rng_state = 10; //fixed seed
```



# Background

### What are soft errors?

Soft error, also known as a transient fault, is a transient bit-flip of a transistor due to external sources such as strikes of alpha particles or cosmic rays. Such faults can cause different failures of computational task execution in cyber-physical systems (CPSs) other than timing failures, e.g., a crash of an execution or incorrect output of a task without being detected. Therefore, to ensure the reliability of CPSs, it is essential to protect critical tasks in CPSs.

### What are SIHFT?

To meet the failure requirements of critical tasks, fault-aware mixed-criticality systems (MCSs) apply software-implemented hardware fault tolerance (SIHFT) to critical tasks. SIHFT provides software-level redundancy to detect (or even correct) soft errors on the tasks. For example, in-thread instruction replication schemes replicate assembly instructions with shadow (redundant) registers, and detect the presence of a soft error by comparing the original and shadow register values at critical points of applications.

### What are re-executions in fault-aware MCSs?

Most of the soft errors on SIHFT-protected tasks are either masked, i.e., do not affect the result, detected by SIHFT, or noticed by the system (crash and hang cases in the presence of watchdog). However, a small number of soft errors may escape SIHFT detection and affect the task output, producing incorrect results without being detected. For further discussion, we define the terms of detected and unrecoverable failures as 1) detected unrecoverable error (DUE), which is a system-visible failure, such as a crash and hang, and 2) silent data corruption (SDC), which is a system-invisible output corruption.

To mitigate DUEs induced by soft errors, fault-aware MCSs allow re-execution(s) of scheduled executions of tasks upon detection of failures. For example, if a system allows up to two additional re-executions, a task can result in DUE only if the first execution results in DUE, and the following two re-executions also result in DUEs. However, such a re-execution strategy upon detection of failures cannot mitigate SDCs, which are undetected failures. Mitigation of SDCs needs redundancy of outputs, such as triple modular redundancy (TMR). For example, if a system forces a task to produce three outputs and proceeds with majority voting among them, the voter will select the correct output unless the majority (two) of the outputs are incorrect, i.e., produced from SDC executions. Note that allowing up to two re-executions for a scheduled execution does not ensure three outputs for majority voting, due to the potential DUEs.



## Future work

The current implementation of our simulator does not consider the logical and physical execution time of the executions and potential timing failures. Future work may implement the schedulability simulator with the current failure simulation based on the timing features supported by Lingua Franca.

In addition, the current SerialTMR only supports a naive TMR, which always executes the task three times. Future work may configure the re-execution based on the fault-aware MCSs. Further, Future work may replace the dummy computation in SingleExecWithFault with other applications.

Finally, future work will support configurable parameters, or example through command-line arguments or external configuration files, to make experiments easier to run and reproduce.

## AI Use Disclosure
We used ChatGPT to assist with parts of the implementation. Generated codes are validated and edited by the authors.

## Contributors

- Hwisoo So (hwisoo.so@knu.ac.kr)
- Hokeun Kim (hokeun@asu.edu)


