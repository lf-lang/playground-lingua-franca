# Patterns

This directory contains Lingua Franca programs representing a number of common design patterns. The naming convention is that each example is in a file named Class_Num_Description.lf, where Class identifies a class of patterns (Chain, Loop, etc.), the Num is a two digit number suggesting the order in which to look at the examples, and Description is a short hint at what particular variant of the pattern the program represents.

## Examples

* [Chain_01_SendReceive](Chain_01_SendReceive.lf): Very simple two-reactor chain.
* [Chain_02_Pipeline](Chain_02_Pipeline.lf): Using a bank to create pipeline.
* [FullyConnected_00_Broadcast](FullyConnected_00_Broadcast.lf): A bank of reactors that broadcast to all others.
* [FullyConnected_01_Addressable](FullyConnected_01_Addressable.lf): A bank of reactors that selectively send to each other.
* [Loop_01_Single](Loop_01_Single.lf): A single reactor sends data to itself.
* [Loop_02_SingleDelay](Loop_02_SingleDelay.lf): A single reactor sends data to itself with delay.

## Library

These programs reuse a collection of library reactors contained in the [lib](lib) subdirectory:

* [SendersAndReceivers](lib/SendersAndReceivers.lf): A collection of reactors that send and receive data.
* [TakeTime](lib/TakeTime.lf): Perform computation for a specified amount of physical time.

If you add to these patterns, please use or extend the library reactors rather than just creating new reactors with similar functionality. Please also fully document your patterns. What do they illustrate? How can the user experiment with them? Also, for each pattern, please add a test in the test subdirectory. Tests should use corresponding test receivers in the lib directory.