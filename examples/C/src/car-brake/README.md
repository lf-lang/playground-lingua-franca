# Car Brake Example

This example illustrates a fundamental tradeoff between consistency and availability as explained in this paper:

> E. A. Lee, R. Akella, S. Bateni, S. Lin, M. Lohstroh, and C. Menard, "Consistency vs. Availability in Cyber-Physical Systems," EMSOFT, October 2023. Earlier version available at: [https://arxiv.org/abs/2109.07771](https://arxiv.org/abs/2109.07771).

The scenario is an ADAS system (advanced driver assistance system), where images from a forward-looking camera are analyzed and may result in a command to sent to a car's braking system.
These programs are sketches in that no actual image analysis or braking is performed, but rather image analysis is simulated by a reaction that sleeps for some time, emulating significant computation time.
The code sleeps for 10ms in our hypothetical example.

In the first version, a deadline violation in a car brake can be triggered when the driver presses the brake pedal and the system is busy analyzing an image generated at a slightly earlier logical time.

The second version forsakes consistency altogether using a Lingua Franca physical connection, which discards timestamps. In this simple example, this may be a reasonable implementation. However, more generally, designs like this where components simply react to the most recent inputs, are deprecated because of the difficulty in testing them. See, for example, [Koopman](https://betterembsw.blogspot.com/2014/09/a-case-study-of-toyota-unintended.html).
For an example of why testing braking systems is important, see [accident where Tesla in self-driving mode applies the brakes for no discernable reason](https://youtu.be/WYpzk6TEViQ).

In the third version, a measured amount of inconsistency is permitted using the LF `after` construct, which introduces a logical delay of 10ms. The given 3ms deadline will be met if computation time, plus network delay, plus clock synchronization add up to less than 13ms.

<table>
<tr>
<td> <img src="img/CarBrake.png" alt="CarBrake" width="400">
<td> <a href="CarBrake.lf">CarBrake.lf</a>: Sketch of an ADAS implementation that emphasizes strong consistency at the expense of availability and therefore will miss deadlines when the brake pedal is pushed while image analysis is being performed.</td>
</tr>
<tr>
<td> <img src="img/CarBrake2.png" alt="CarBrake2" width="400">
<td> <a href="CarBrake2.lf">CarBrake2.lf</a>: Sketch of an ADAS implementation that forgoes consistency and determinacy in order to achieve high availability. This program should not miss deadlines when the brake pedal is pushed while image analysis is being performed.</td>
</tr>
<tr>
<td> <img src="img/CarBrake3.png" alt="CarBrake3" width="400">
<td> <a href="CarBrake3.lf">CarBrake3.lf</a>: Sketch of an ADAS implementation that emphasizes strong consistency at the expense of availability and therefore will miss deadlines when the brake pedal is pushed while image analysis is being performed.</td>
</tr>
</table>