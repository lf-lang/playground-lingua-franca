# Triple Modular Redundancy in Time-Sensitive Systems

## Overview

This playground example demonstrates the scheduling of the triple modular redundancy (TMR) mechanism in time-sensitive systems.

## Description

This example provides a high-level, abstracted failure simulation on Lingua Franca (LF) to simulate the effect of the re-execution for fault-aware MCSs under potential DUEs and SDCs. Specifically, this example models a naive TMR, which re-executes a task twice, i.e., one execution and two additional re-executions, and proceeds with the majority voting among completed executions. Each execution in this example is classified as benign (success), DUE, or SDC with given probabilities. These probabilities are derived from a specified fault rate and task characteristics following the failure failure models from PREFACE, a state-of-the-art fault-aware MCS approach. Further, this example provides N-modular redundancy (NMR), e.g., 5-Modular Redundancy, and parallel execution of redundant executions on LF.

### How to Use?

You can compile `TMR.lf` with `lfc` as follows.

First, move to the root directory of the Lingua Franca Playground repository. Then run:

```bash
lfc examples/C/src/triple-modular-redundancy/TMR.lf
```

After compilation, execute the generated program with:

```bash
./examples/C/bin/TMR
```

Note: this example requires Lingua Franca version 0.12.0 or later.

### How to Configure?

Currently, this example does not support command-line parameters. To change the configuration, manually edit the parameters defined in `TMR.lf` before compilation.

You can configure DUE and SDC probabilities per execution by modifying the following variables in preamble:

```
static const double due_per_exec = 0.3;
static const double sdc_per_exec = 0.3;
```

In Default, this example simulates the naive TMR. You can simulate different level of redundancy  by modifying the following variable in SerialTMR reactor:

```
state max_number_of_executions: int = 3
```

For exmaple, if you set max_number_of_executions as 4, this example will simulate naive quadruple modular redundancy with first execution and additional three re-executions.
	
You can change the random seed with the following variable in preamble:

```
static uint64_t rng_state = 10; //fixed seed
```



## Background

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

Finally, future work will  support configurable parameters, or example through command-line arguments or external configuration files, to make experiments easier to run and reproduce.

## Contributors

- Hwisoo So (hwisoo.so@knu.ac.kr)
- Hokeun Kim (hokeun@asu.edu)


