# Rhythm

These examples illustrate the use of **audio drivers** on Mac and Linux and the use of **ncurses** to create old-fashioned terminal-based user interfaces.
<table>
<tr>
<td> <img src="img/Sound.svg" alt="Sound" width="100%"> </td>
<td> <a href="Sound.lf">Sound.lf</a>: This example plays a simple sound (a drum generating a merengue rhythm). This illustrtes the use of the PlayWaveform reactor, which illustrates the use of the <a href="https://www.lf-lang.org/reactor-c/d1/dcb/audio__loop_8h.html"> audio\_loop.h</a> and <a href="https://www.lf-lang.org/reactor-c/d3/d8a/wave__file__reader_8h.html"> wave\_file\_reader.h</a> APIs.</td>
</tr>
<tr>
<td> <img src="img/SensorSimulator.svg" alt="SensorSimulator" width="100%"> </td>
<td> <a href="SensorSimulator.lf"> SensorSimulator.lf</a>: This example illustrates the use of the <a href="https://www.lf-lang.org/reactor-c/dc/de9/sensor__simulator_8h.html"> sensor\_simulator.h</a> API, which turns a terminal into a rudimentary user interface using ncurses. This enables simple  text-and-keyboard interactions with Lingua Franca programs in a portable way.</td>
</tr>
<tr>
<td> <img src="img/Rhythm.svg" alt="Rhythm" width="100%"> </td>
<td> <a href="Rhythm.lf"> Rhythm.lf</a>: This example combines the two previous examples to create a simple interactive program that reacts to keyboard inputs to produce sounds with various rhythms and tempos.</td>
</tr>
<tr>
<td> <img src="img/RhythmDistributed.svg" alt="RhythmDistributed" width="100%"> </td>
<td> <a href="RhythmDistributed.lf"> RhythmDistributed.lf</a>: This example creates a federated version where two rhythm generators run in two separate processes and coordinate their tempos and rhythms under user control from either process.</td>
</tr>
<tr>
<td> <img src="img/RhythmDistributedNoUI.svg" alt="RhythmDistributedNoUI" width="100%"> </td>
<td> <a href="RhythmDistributedNoUI.lf"> RhythmDistributedNoUI.lf</a>: This variant of the previous example has no user interface, which makes it easier to launch (no need to launch the individual federates in different windows; just use the launch script). It can be used to test clock synchronization. The `test-offset` clock-sync option sets a clock offset, which the clock synchronization mechanism compensates for. If you turn clock synchronization off, you will hear the offset.</td>
</tr>
</table>