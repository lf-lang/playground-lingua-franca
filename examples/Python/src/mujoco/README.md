# Mujoco Robot Simulation

This directory shows how to use the `mujoco-py` package, subclassing the base classes provided there to realize
simulations of robots defined in the `mujoco_menagerie` submodule.

## Prerequisites

[MuJoCo](https://mujoco.org) (Multi-Joint dynamics with Contact) is a physics-based simulation engine with graphics and animation for the Python target. To install Python MuJoCo:

```sh
python3 -m pip install mujoco
```

## Programs

<table>
<tr>
<td> <img src="img/PandaDemo.svg" alt="PandaDemo" width="400">
<td> <a href="PandaDemo.lf"> PandaDemo.lf</a>: Simulation of the Frank Emika Panda going through its motions. This demo also illustrates how to read a parameter file (CSV format) to set parameters.</td>
</tr>
</table>
