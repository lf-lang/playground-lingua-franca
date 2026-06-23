# GNSS Pipeline

This example models a predictable GNSS (Global Navigation Satellite System) data pipeline in Lingua Franca (LF).

It demonstrates how deterministic timing can be achieved across a sensor/peripheral pipeline:

- `GNSSReceiver`: generates RTCM correction data every 100 ms, simulating a multi-constellation GNSS receiver.
- `UARTDriver`: models a fixed-rate UART peripheral with a 16-byte FIFO and interrupt-driven transmission.
- `DataForwarder`: uses an 18 ms periodic task to drain a 1000-byte ring buffer and forward data to a network.
- `PositionEngine`: simulates a SoC-side position processing engine that receives the forwarded RTCM stream.
- `LatencyMonitor`: observes end-to-end timing and physical/logical jitter.

Key predictability factors in this example:

- 100 ms GNSS epoch period (10 Hz update rate)
- Fixed UART baud rate of 460800 bps
- 16-byte hardware FIFO buffer for UART interrupts
- 1000-byte software ring buffer for data buffering
- 18 ms periodic forwarding task to avoid unbounded queue growth

## Files

- `GNSSPipelineAsynchronous.lf`: the pipeline with the UART interrupt modeled as a **physical
  action** (the realistic, asynchronous model — see below).
- `GNSSPipelineSynchronous.lf`: an otherwise-identical companion that models the UART interrupt as a
  **logical action** (synchronous to logical time), provided for a side-by-side comparison of the
  two timing semantics.

The two names map to the action type used for `hw_interrupt`:

| File                          | `hw_interrupt` action | Meaning                                                        |
| ----------------------------- | --------------------- | ------------------------------------------------------------- |
| `GNSSPipelineAsynchronous.lf` | `physical action`     | tag from the wall clock — models a true asynchronous interrupt |
| `GNSSPipelineSynchronous.lf`  | `logical action`      | tag from logical time — drift-free and fully reproducible      |

## How to build and run

1. Open a terminal in `examples/C/src/gnss-pipeline`.
2. Compile with the Lingua Franca compiler (these are *federated* programs, so the compiler also
   generates and builds the RTI):

   ```bash
   lfc GNSSPipelineAsynchronous.lf
   lfc GNSSPipelineSynchronous.lf
   ```

3. Run the generated launcher (it starts the RTI and all federates). From the package root
   `examples/C`:

   ```bash
   bin/GNSSPipelineAsynchronous   # physical-action version
   bin/GNSSPipelineSynchronous    # logical-action version
   ```

   Both use `keepalive: true` and have no `timeout`, so they run until interrupted (Ctrl-C).

## Asynchronous vs. synchronous (physical vs. logical action)

The single line that differs between the two programs is the declaration of `hw_interrupt` inside
`UARTDriver`:

```
physical action hw_interrupt: int   // GNSSPipelineAsynchronous.lf
logical  action hw_interrupt: int   // GNSSPipelineSynchronous.lf
```

Everything else — parameters, reactions, connections, and log statements — is identical, so any
difference in behavior is attributable solely to the action type.

### Why the asynchronous version uses a *physical* action

A UART FIFO interrupt is a genuinely **asynchronous, hardware-driven** event. Modeling it with a
physical action is the right choice for three reasons:

1. **Tag assignment from the physical clock.** When `lf_schedule` fires a physical action, the
   event's tag is taken from the *physical* (wall-clock) clock (`physical_time + offset`), not from
   the current logical time. This faithfully captures the reality that the interrupt's arrival time
   is determined by hardware (baud rate, FIFO fill) and carries real hardware/OS jitter. The
   physical action is, in the paper's words, *"the boundary between unpredictable hardware time and
   deterministic logical execution."*
2. **Safe injection of external events.** A physical action is the **only** thread-safe way to push
   an event into the LF runtime from *outside* a reaction — e.g., from a real ISR or sensor
   callback. A logical action can only be scheduled from within a reaction. So in a real deployment,
   the interrupt path *must* be a physical action.
3. **Keepalive semantics.** Physical actions tell the runtime that events may still arrive from the
   physical world, which is why the program declares `keepalive: true`.

### What changes with the synchronous (*logical* action) version

In `GNSSPipelineSynchronous.lf`, `lf_schedule_int(hw_interrupt, offset, ...)` assigns the tag in
**logical** time as `current_logical_time + offset`. Consequences:

- **Drift-free cadence.** Each interrupt lands exactly `offset` after the previous one. The
  execution time of the scheduling reaction does *not* leak into the next tag.
- **Fully reproducible logical intervals.** The inter-interrupt intervals (and therefore the whole
  causal trace) are independent of physical execution and identical across runs and machines.
- **Loss of realism.** It no longer models a truly asynchronous interrupt, and it could not be
  driven from a real external ISR thread. It is an *idealized* model, useful for analysis.

### Observed behavior (federated runs, ~5 s each on this machine)

Both versions produce the **same data** — identical RTCM byte counts, ring-buffer evolution, and
`{18, 90, 108} ms` forwarding pattern — confirming they agree on *what* happens. The difference is
in the *timing of the interrupt burst*, measured as the logical-time span over which one epoch's 20
interrupts occur:

| Version                       | Epoch-1 interrupt-burst span | Steady-state span | Cause                                                    |
| ----------------------------- | ---------------------------- | ----------------- | ------------------------------------------------------- |
| **Synchronous** (logical)     | ~0.6 ms (1 ms bucket)        | ~0.6 ms           | tags advance by exactly the offset → zero drift         |
| **Asynchronous** (physical)   | up to ~3–4 ms (cold start)   | ~1 ms             | each tag re-reads the wall clock → absorbs real overhead |

The synchronous version compresses every epoch's burst into the same ~0.6 ms regardless of run or
system load. The asynchronous version lets the burst **smear** — most visibly at cold start, where
per-reaction overhead accumulates across the 20 self-rescheduled interrupts and pushes the burst out
to ~4 ms. That smear is a function of physical conditions, not of the model — which is exactly the
property a physical action is meant to expose.

> Notes:
> - On a lightly loaded machine the two versions are nearly indistinguishable in steady state; the
>   asynchronous (physical-action) version's guarantees are *weaker*, not its typical behavior worse.
>   Under load or on a slower target, the asynchronous version diverges while the synchronous version
>   cannot.
> - Absolute printed timestamps shift between runs in *both* versions because a federated program's
>   start instant is derived from the wall clock. The synchronous model's reproducibility is about
>   logical *intervals* relative to start, not the absolute start instant.
> - The model uses `1000000 / 460800` (integer division) for the per-byte time, which truncates to
>   2 µs/byte (32 µs per 16-byte interrupt) instead of the intended ~21.7 µs/byte (~347 µs). This
>   does not affect the asynchronous-vs-synchronous comparison, but it does make the interrupt burst
>   much shorter than the paper's analytical figures.

## What this example shows

- Event-driven modeling of a GNSS receiver and UART hardware.
- Physical actions and time-triggered behavior for predictable real-time systems.
- The semantic contrast between physical (asynchronous) and logical (synchronous) actions for the
  same interrupt source.
- Buffering and periodic task design for embedded data pipelines.
- Monitoring of data flow and timing behavior for analysis.
