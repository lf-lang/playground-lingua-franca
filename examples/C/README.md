# C Examples

The examples are grouped into unfederated, federated, and embedded and listed in alphabetical order within each category.

## Unfederated

* [Browser UIs](src/browser-ui/README.md): How to create a browser-based UI for an LF program.
* [Car Brake](src/car-brake/README.md): Sketch of ADAS system illustrating the CAL theorem.
* [Deadlines](src/deadlines/README.md): Uses of deadlines in Lingua Franca.
* [Furuta Pendulum](src/modal_models/FurutaPendulum/README.md): A controller and simulation illustrating a modal reactor.
* [Keyboard](src/keyboard/README.md): Responding to keyboard input using ncurses.
* [Patterns](src/patterns/README.md): Common communication patterns.
* [Reflex Game](src/reflex-game/README.md): Interactive timed game.
* [Rhythm](src/rhythm/README.md): Sound generation and terminal user interface demos.
* [Rosace](src/rosace/README.md): Aircraft controller illustrating periodic systems with multiple periods.
* [SDV](src/sdv/README.md): Software defined vehicle sketch integrating user input, a web display, and sound.
* [Simulation](src/simulation/README.md): Using Lingua Franca for simulation.
* [Synchrophasors](src/synchrophasors/README.md): Illustration of the value of timestamp semantics.
* [Train Door](src/train-door/README.md): Train door controller from a verification paper.

## Federated

These examples illustrated federated execution, where each top-level reactor becomes its own program that can be deployed across containers and networked machines.

* [Chat](src/chat/README.md): Simple federated chat application with two users (run manually).
* [Distributed](src/distributed/README.md): Basic federated hello-world examples.
* [Distributed STA and STAA](src/distributed-sta/README.md): How to set STA and STAA in federated/decentralized.
* [Distributed Time](src/distributed-time/README.md): Federated, decentralized, with deadlines.
* [MQTT](https://github.com/lf-lang/mqtt-c): Interacting with MQTT publish-and-subscribed (moved to a library repo).
* [Leader Election](src/leader-election/README.md): Federated fault-tolerant system with leader election.
* [Rhythm](src/rhythm/README.md): Sound generation and terminal user interface demos.
* [Shared Memory](src/shared-memory/README.md): Using shared memory to exchange large data objects between federates. 
* [Watchdog](src/watchdog/README.md): Federated illustration of watchdogs.
* [Zero-delay Cycles](src/zero-delay-cycles/README.md): Federated patterns with zero-delay cycles.


## Embedded

These examples illustrate the use of LF for programming embedded processors. They require additional hardware and software to compile and run.

* [LED Matrix](src/led-matrix/README.md): LED matrix driver on Raspberry Pi Pico.
