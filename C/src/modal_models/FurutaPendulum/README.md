# Furuta Pendulum

A [Furuta pendulum](https://en.wikipedia.org/wiki/Furuta_pendulum) is a classic control system problem often used to teach feedback control. 
It consists of a vertical shaft driven by motor, a fixed arm extending out at 90 degrees from the top of the shaft, and a pendulum at the end of the arm.
The goal is to rotate the shaft to impart enough energy to the pendulum that it swings up, to then catch the pendulum and balance it so that the pendulum remains above the arm.
Each of these steps requires a different control behavior, which makes a controller a prime candidate for a modal model.

The example here replicates [a solution by Eker et al.](https://ptolemy.berkeley.edu/papers/02/IFAC/IFAC.pdf)
It uses a simple forward-Euler simulation of an actual pendulum and the control logic provided by Eker et al. to implement a modal controller in Lingua Franca.

The program is configured to automatically plot the results of the simulation.


## Prerequisites

The executable examples use a custom build script that compiles the program, as usual, but also executes the program and processes its output to generate and open a plot. 
It relies on [cmake](https://cmake.org/) and [gnuplot](http://www.gnuplot.info/) being available on the PATH.
The script is not compatible to Windows systems.

The script automatically opens the resulting plot.
By default it uses the 'open' command (MacOS).
You may need to change the last argument passed to the build script to the name of a PDF viewer installed on your system (e.g. 'xdg-open' on Linux).


## Summary of the Examples

* **[FurutaPendulum](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/modal_models/FurutaPendulum/FurutaPendulum.lf)**:
This is the primary example. 
It uses the modal controller and the simulation to illustrate a 3 second run of a Furuta pendulum and plots the results.

* **[FurutaPendulumDisturbance](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/modal_models/FurutaPendulum/FurutaPendulumDisturbance.lf)**:
In this example, the pendulum simulation is run for 5 seconds with an external disturbance introduced at 3 seconds that will move the pendulum out of its upright position and requires re-balancing of the controller.


### Involved Models

* **[PendulumController](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/modal_models/FurutaPendulum/PendulumController.lf)**:
The controller implementation using modes to separate the control logic.

* **[PendulumSimulation](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/modal_models/FurutaPendulum/PendulumSimulation.lf)**:
The simulation implementation based on a simple forward-Euler simulation by Eker et al.

* **[Print](https://github.com/lf-lang/examples-lingua-franca/blob/main/C/src/modal_models/FurutaPendulum/Print.lf)**:
A utility reactor that prints the state of the pendulum into a csv file.
