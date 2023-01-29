# Periodic Programs in Lingua Franca

Although Lingua Franca programs are, in general, event driven, it is easy to create periodic real-time programs with a multiplicity of periods. These examples illustrate how to do this. Each of the reactors is triggered by a timer that specifies its period. If its reactions read data from input ports, those ports are declared in the [*uses* field](https://www.lf-lang.org/docs/handbook/inputs-and-outputs?target=c#triggers-effects-and-uses) of the reactor signature rather than the *triggers* field. The dependencies are automatically handled by the LF code generator so that upstream reactions that execute in the same tag will always complete before a downstream reaction that uses its data.

Note that these examples rely on the persistence of port data that is implemented in the C target. That is, when reading data from an input port declared in the *uses* field, the *most recently received* data will be provided even if, at the current tag, that port is absent. These examples would have to be designed in a different way in a target that does not support persistent data, such as the Cpp target.

* [AircraftSimulator.lf](AircraftSimulator.lf): A forward Euler simulation of aircraft dynamics (from teh ROSACE case study).
* [RosaceController.lf](RosaceController.lf): A controller for an aircraft (from the ROSACE case study).
* [Rosace.lf](Rosace.lf): A composition of the aircraft and controller, illustrating periodic sample rates interacting.