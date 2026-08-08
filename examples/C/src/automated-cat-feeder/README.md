<h1 align="center">
  <br>
  Automated Cat Feeder
  <br>
</h1>

<h4 align="center">
  A weight-sensing pet feeder built with 
  <a href="https://www.lf-lang.org/" target="_blank">Lingua Franca</a>.
</h4>

<p align="center">
  <img src="assets/cat_feeder.png" width="400">
</p>


---

## Overview

This project is an elementary automated pet feeder designed to run on a Raspberry Pi 4. 
If the food weight is below a certain threshold, the motor is triggered dispensing more food using an archimedes screw.

*Assumptions*
- Goal food weight: 50 - 60 grams
- Data pin= 5, Clock pin= 6
- Empty bowl placed to begin with
- Bowl Calibration Base Weight: 44 grams


The goal is to ensure a pet always has the ideal amount of food (50-60 grams) while demonstrating how deterministic, reactive programming can be used to handle noisy sensor data, hardware interrupts, and mechanical dispensing logic safely.

---
## Usage
To compile and run this application, ensure you have Lingua Franca installed. From your command line:

```bash
# 1. Compile the main reactor
lfc feed.lf

# 2. Run the program (sudo is required for GPIO hardware priority)
sudo bin/feed
```
---

## Features

- **Real-Time Weight Monitoring:** Uses an HX711 amplifier to poll a load cell every second.
- **Median Data Filtering:** Takes 10 samples per reaction and calculates the median to smooth out noisy hardware readings.
- **Automated Dispensing:** Uses a modal model to switch a stepper motor from `IDLE` to `DISPENSE` when food drops below 50g.
- **Safety Cooldowns:** Detects anomalous readings (e.g., if a cat steps on the bowl) and automatically triggers a 10-second measurement pause to prevent overfeeding or motor burn-out.
- **Startup Calibration:** Interactive terminal prompt to establish a zero-weight offset for the specific bowl being used.

---

## Hardware Requirements

- **[Raspberry Pi 4 Model B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)** 
- **[HX711 Load Cell Amplifier(1 KG)](https://www.amazon.com/dp/B0BLNQZRBD?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)**
- **[Load Cell](https://www.amazon.com/dp/B0BLNQZRBD?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)** 
- **Stepper Motor**
- **[Stepper Hat B](https://www.waveshare.com/wiki/Stepper_Motor_HAT_(B)?srsltid=AfmBOorD0cHVk7AqhdQ46iFxuBOxcegc7QxJ7O0b3TRa4oe1HzmYGizS)**

---

## Software Requirements

- **Lingua Franca compiler** (`lfc`)
- **GCC toolchain** (`gcc`) for the C target
- **[WaveShare BCM2835 C Library](https://www.waveshare.com/wiki/Stepper_Motor_HAT_(B)#BCM2835)** 
- **[MikeM BCM2835 C Library](https://www.airspayce.com/mikem/bcm2835/)** 
- **[HX711 C Library](https://github.com/gandalf15/HX711)** 


### Custom Library Modifications
To ensure real-time safety and hardware stability on the ARM-based Raspberry Pi, the standard `hx711.c` library was modified for this project:
* **Memory Barriers:** Added `__sync_synchronize()` around direct GPIO register access to prevent CPU instruction reordering.
* **Precise Timing:** Replaced empty `for' loop delays with explicit `usleep(1)` hardware delays.
* **Memory Management:** Stripped out dynamic memory allocation (`malloc`/`free`) to prevent fragmentation during long uptimes.

---

## Mechanical Requirements
- **Archimedes Screw** 
- **Container** 

---

### Components(Reactors)
<table>
  <tr>
    <td> <img src="assets/feed.png" alt="Feed Reactor" width="800"></td>
    <td> <strong><a href="feed.lf">feed.lf</a></strong>: The main reactor that instantiates the motor and loadcell components. It controls the startup sequence, prints the terminal interface, and acts as the central hub by routing the <code>low_food</code> signal from the scale to trigger the motor's dispensing logic.</td>
  </tr>
  <tr>
    <td> <img src="assets/motor.png" alt="Motor Reactor" width="800"></td>
    <td> <strong><a href="lib/motor.lf">motor.lf</a></strong>: Manages the physical food dispensing mechanism. It uses a modal model to switch between <code>IDLE</code> and <code>DISPENSE</code> states, activating the HR8825 stepper motor driver to rotate a specific number of steps when commanded to feed.</td>
  </tr>
  <tr>
    <td> <img src="assets/loadcell.png" alt="Loadcell Reactor" width="800"></td>
    <td> <strong><a href="lib/loadcell.lf">loadcell.lf</a></strong>: Handles the HX711 scale sensor. It takes median-filtered weight samples every two seconds, manages interactive startup calibration, evaluates the food level against ideal thresholds (50-60g), and enforces a 10-second safety cooldown if it detects anomalous readings.</td>
  </tr>
</table>

### Demo
[Demo Video](https://youtube.com/shorts/B0-c7XisVDo)

### Contributor
[Irene Fahndrich](https://github.com/IreneMarciana)



