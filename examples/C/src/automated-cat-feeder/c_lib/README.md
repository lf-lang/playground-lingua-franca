<h1 align="center">
  <br>
    C Libraries Used Overview
  <br>
</h1>

## bcm2835
Debug, DEV_config, and HR8825 are from the [waveshare/bcm2835 Library](https://www.waveshare.com/wiki/Stepper_Motor_HAT_(B)#BCM2835) from Waveshare for the Stepper Motor Hat B.

bcm2835 is from another [mikem/bcm2835 Library](https://www.airspayce.com/mikem/bcm2835/). This was only included to facilitate compilation on Ubuntu 24 on Github, since the library used for the Stepper Motor Hat is Raspberry Pi specific.

- **Debug**: Debug printing statements structure defined.
- **DEV_config**: Handles pin setup and timing delays for the stepper motor hardware. 
- **HR8825**: Contains functions for motor selection, direction, microstepping configuration, and step-based movement execution for stepper motor.

- **bcm2835**: Defines registers, constants, and functions for GPIO, SPI, I2C, PWM, and memory-mapped peripheral control; in standard system directory for Pi OS, but not Ubuntu.



## hx711
gb_common and hx711 are from the [HX711 C Library](https://github.com/gandalf15/HX711).
- **gb_common**: Provides low-level memory-mapped GPIO setup and utility functions for direct hardware access on the Pi, including pin control and timing loops.
- **hx711**: Handles GPIO signaling, data reading, calibration, and weight measurement with added memory barriers for reliable hardware interaction with loadcell amplifier HX711. Upgraded to work with Raspberry Pi 4 with memory barriers.