<h1 align="center">
  <br>
  Automated Cat Feeder
  <br>
</h1>

## Overview
placeholder

## Components
<table>
  <tr>
    <td> <a href="lib/motor.lf">motor.lf</a>: This is the reactor for the stepper motor, using modal reactors for modes IDLE, DISPENSE, and ERROR.</td>
  </tr>
  <tr>
    <td> <a href="lib/loadcell.lf">loadcell.lf</a>: This reactor handles the loadcell reaction, which is triggered every 1 second and takes 10 samples. It outputs a boolean determining whether the food bowl is above or below a certain threshold.</td>
  </tr>
  <tr>
    <td> <a href="feed.lf">feed.lf</a>: This is the main reactor, combining loadcell measurement and motor modal logic for feeding process.</td>
  </tr>
</table>











To be done:
- overview
- hardware
- software(mod bcm + hx711 changes)
- demo
- usage
