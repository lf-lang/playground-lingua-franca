# ACAS Xu Collision Avoidance

This demonstrator application realizes an **ACAS Xu** (airborne collision avoidance system for unoccupied vehicles) described in the following paper:

>  Arthur Clavière, Laura Altieri Sambartolomé, Eric Asselin, Christophe Garion, ans Claire Pagetti, "[Verification of machine learning based cyber-physical systems: a comparative study](https://doi.org/10.1145/3501710.3519540)," International Conference on Hybrid Systems: Computation and Control (HSCC), May 2022, Pages 1–16.

There are two examples provided, both of which use the same aircraft plant models and the same controller models. In both cases, one aircraft is the _ownship_ and the other is an _intruder_, and both move in a horizontal plane:

<img alt="aircraft properties" src="img/state.png" width="200">

The controller uses a set of five pre-trained neural networks to determine the turn angle to apply. The plots from each example should look like the following:

<img alt="aircraft trajectories" src="img/ACASXuPlot.png" width="360">
<img alt="aircraft trajectories" src="img/ACASXu2Plot.png" width="400">

In both cases, the intruder aircraft is in red; the own aircraft in blue; and both aircraft start at the bottom and move up. In the left plot, the intruder aircraft is not equipped with an ACAS controller and maintains a constant trajectory, oblivious to the possible collision. In the second plot, both aircraft are equipped with ACAS.

## Programs

<table>
<tr>
<td> <img src="img/ACASXu.png" alt="ACAS Xu" width="400">
<td> <a href="ACASXu.lf">ACASXu.lf</a>: Two aircraft, one of which is equipped with neural-network-based ACAS system.</td>
</tr>
<tr>
<td> <img src="img/ACASXu2.png" alt="ACAS Xu 2" width="400">
<td> <a href="ACASXu2.lf">ACASXu2.lf</a>: Two aircraft, both of which are equipped with neural-network-based ACAS systems.</td>
</tr>
</table>

The Python code on which this is based was originally written by Arthur Clavière and is distributed (see [https://svn.onera.fr/schedmcore/branches/](https://svn.onera.fr/schedmcore/branches/)) under an [LGPL version 3 license](https://www.gnu.org/licenses/lgpl-3.0.html), so this directory also has an LGPL [LICENSE](LICENSE.md), which is non-standard compared to other LF code. The pre-trained neural networks were developed at Stanford University as part of the Reluplex project and are released under the [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).  The authors of the neural nets are listed in the [AUTHORS](code/src/systems/acasxu/nnets/AUTHORS) file.



