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

## File

- `GNSSPipeline.lf`: Lingua Franca source for the complete GNSS pipeline

## How to build and run

1. Open a terminal in `examples/C/src/GNSSPipeline`
2. Compile the example using the Lingua Franca compiler:

   ```bash
   lfc GNSSPipeline.lf
   ```

3. Run the generated executable. By default, the C target produces a binary named `GNSSPipeline`:

   ```bash
   ./GNSSPipeline
   ```

## What this example shows

- Event-driven modeling of a GNSS receiver and UART hardware.
- Physical actions and time-triggered behavior for predictable real-time systems.
- Buffering and periodic task design for embedded data pipelines.
- Monitoring of data flow and timing behavior for analysis.
