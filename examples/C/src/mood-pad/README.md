# MoodPad – Interactive Sensor-Driven MIDI Synth

## Overview

MoodPad is an interactive MIDI synthesizer project built with **Lingua Franca (LF) targeting C**, designed to run on a Raspberry Pi. The system uses environmental sensors to dynamically control musical output:

- **BME280**: Temperature, pressure, and humidity – used to select root notes and octaves.  
- **BH1750**: Ambient light – used to choose between major and minor scales.  
- **GT911**: Capacitive touch panel – triggers notes based on touch position.  
- **FluidSynth**: MIDI synthesis for generating piano sounds.

The goal is a **sensor-driven musical instrument** where the environment directly influences the pitch, key, and dynamics of the generated notes, as shown below:

<img width="613" height="562" alt="Screenshot 2025-12-15 at 9 54 19 AM" src="https://github.com/user-attachments/assets/c498cca0-86e0-41c2-a4bc-eaa1d78bf38a" />

---

## Features

- Temperature-controlled root note and octave selection  
- Light-controlled scale mode (major/minor)  
- Touch-sensitive note triggering with velocity mapping  
- Real-time MIDI output via ALSA to FluidSynth or other synthesizers  

---

## Hardware Requirements

- [Raspberry Pi (tested on Pi 3B+)](https://www.digikey.com/en/products/detail/raspberry-pi/SC0073/8571724?gclsrc=aw.ds&gad_source=1&gad_campaignid=20228387720&gbraid=0AAAAADrbLli9GnIE8yvimHJOwvN2NmGPy&gclid=CjwKCAiA3fnJBhAgEiwAyqmY5cdtQ73Fm_pR7ht6KIIG0o8OEyC0vPacOd7sQgWMNwt7QzOIJpq96BoCRJEQAvD_BwE)
- [BME280 temperature/humidity sensor](https://www.adafruit.com/product/2652?gad_source=1&gad_campaignid=21079227318&gbraid=0AAAAADx9JvS43_-m5zEAgJ2DoWPzZ7wdL&gclid=Cj0KCQjw8eTFBhCXARIsAIkiuOydzZ0Ovo-drdjscpSFsik74wdNou4V4kaXOjs-ulsvpnLHNiQrwF8aAujtEALw_wcB)
- [BH1750 light sensor](https://www.adafruit.com/product/4681?gad_source=1&gad_campaignid=21079227318&gbraid=0AAAAADx9JvS43_-m5zEAgJ2DoWPzZ7wdL&gclid=Cj0KCQjw8eTFBhCXARIsAIkiuOzbuVQEqIr4v3gIOZK4EqyDz-LWlgPaNhcWKOFixgmkw9fUHa_NuQcaArHiEALw_wcB)  
- [GT911 capacitive touch sensor (or compatible touchscreen)](https://www.buydisplay.com/8-inch-capacitive-touch-panel-with-controller-gt911-5-point-multi-touch?srsltid=AfmBOop_zEfzfRkqseXNPjFCipHO50TSfBZCrgrjhLCAnY2NmrBodM-jIqk)
- Audio output device (headphones or HDMI audio)  
- SoundFont `.sf2` file for FluidSynth

---

## Software Requirements

- **Lingua Franca compiler** (`lfc`)  
- GCC toolchain (`gcc`) for C target  
- [**ALSA** libraries](https://github.com/alsa-project/alsa-lib) (`libasound2-dev`)
- [**FluidSynth** libraries](https://www.fluidsynth.org/)
  - **FluidSynth** (`fluidsynth`) for MIDI synthesis  
  - **FluidSynth** (`gpiod`) for gpio interactions  


---

## Setup

### 1. Install dependencies

```bash
sudo apt update
sudo apt install -y build-essential libasound2-dev fluidsynth gpiod
```

### 2. Compile LFC
```bash
lfc Moodpad.lfc
```

---

## Contributor

[Elise Macabou](https://github.com/elisemacabou)

---
