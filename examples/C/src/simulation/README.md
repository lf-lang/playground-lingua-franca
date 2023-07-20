# Using Lingua Franca for Simulation

This set of examples shows how to use LF to simulate random discrete-event processes.

## Repeatable Behavior

Because of its discrete-event semantics, Lingua Franca can be quite useful for building simulations. Random numbers and events occurring at random times are usually central to effective simulations. Random numbers can also be useful for creating regression tests. In all of these use cases, it is usually important to be able to get repeatable behavior. This requires controlling the seeds of the random number generators.

The key challenge here is that we will often need for each distinct reactor that generates random numbers to have repeatable behavior. As a consequence, each such reactor should have its own seed and distinct reactors cannot share the same random number generator. If they do, then the behavior that emerges may depend on accidental scheduling decisions.

## Examples

* [MemoryHierarchy](MemoryHierarchy.lf): A simulation of a memory hierarchy such as a caching system.
* [PoissonProcess](PoissonProcess.lf): Generate two Poisson processes, one repeatable and one not.

## Library Reactors

Each of these examples uses reactors defined in the [lib](lib) directory:

* [PoissonClock](lib/PoissonClock.lf): This produces output events according to a Poisson process.
* [Random](lib/Random.lf): A base class that manages seeds and provides utility functions for generating random numbers.
* [RandomDelay](lib/RandomDelay.lf): Delay an input value by a time given by an exponential random variable.
