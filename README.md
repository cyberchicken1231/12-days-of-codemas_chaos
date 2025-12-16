# 12 Days of Codemas - Chaos System

A progressive MicroPython chaos system for Raspberry Pi Pico H, building up through 12 days of hardware additions. Each day adds new sensors and inputs that influence a unified chaos organism combining quantum oscillators, Dirac spinor modulation, logistic map chaos, and XOR entropy fusion.

## Hardware Requirements

- **Raspberry Pi Pico H** (RP2040)
- MicroPython firmware installed
- Half-size breadboard
- Components listed below by day

---

## Day-by-Day Build Guide

### Day 1: Basic Setup
**File:** `day-1.py`

**Components:**
- Raspberry Pi Pico H
- Breadboard
- USB cable

**Pin Mapping:**
- GPIO 25: Built-in LED

**Features:**
- Simple LED blink test
- Verifies Pico is working

---

### Day 2: LED Chaos Foundation
**File:** `day-2.py`

**Components:**
- 3x LEDs (Red, Amber, Green)
- 3x 330Ω resistors
- 4x Jumper wires

**Pin Mapping:**
- GPIO 18: Red LED
- GPIO 19: Amber LED
- GPIO 20: Green LED
- ADC27: Floating noise source

**Features:**
- Dirac spinor modulation (4 components)
- Logistic map chaos
- Quantum Rabi oscillator
- Mutation with drift
- 5 entropy sources: noise, chaos, mutation, quantum, spinor

---

### Day 3: Button Input
**File:** `day-3.py`

**Components:**
- 3x Tactile buttons
- 3x Button caps
- Mini breadboard
- 7x Jumper wires

**Pin Mapping:**
- GPIO 13: Button 1
- GPIO 8: Button 2
- GPIO 3: Button 3
- (All with PULL_UP resistors)

**Features:**
- Buttons modulate Dirac spinor phase and amplitude
- Button pressure affects chaos parameters
- 6 entropy sources: adds button input to Day 2

---

### Day 4: Potentiometer Speed Control
**File:** `day-4.py`

**Components:**
- 1x 10KΩ potentiometer
- 1x Potentiometer knob
- 3x Jumper wires

**Pin Mapping:**
- ADC26 (GPIO 26): Potentiometer

**Features:**
- **EXTREME speed control**: 0.5 sec/loop (slow) to 0.1 ms/loop (very fast)
- Pot adds aggressive chaos influence
- Time dilation effect on loop timing
- Bit-flip energy control

---

### Day 5: Audio Feedback
**File:** `day-5.py`

**Components:**
- 1x Piezo transducer with jumper wires

**Pin Mapping:**
- GPIO 15: Piezo PWM

**Features:**
- Chaos-driven audio (200-4000 Hz)
- Frequency modulated by all chaos sources
- Duty cycle varies with system state
- Audible representation of entropy

---

### Day 6: Light Sensing
**File:** `day-6.py`

**Components:**
- 1x Phototransistor
- 1x 10KΩ resistor
- 3x Jumper wires

**Pin Mapping:**
- ADC28 (GPIO 28): Phototransistor

**Features:**
- Light level → optical entropy bits
- Brightness influences bit-flip probability
- 7 entropy sources: adds optical randomness

---

### Day 7: Motion Detection
**File:** `day-7.py`

**Components:**
- 1x Mini PIR sensor
- 3x Jumper wires

**Pin Mapping:**
- GPIO 22: PIR motion sensor

**Features:**
- Motion detection with edge triggering
- Burst mode on motion detection
- PIR state XORed with LED outputs
- Excites piezo frequency on motion

---

### Day 8: Temperature Sensing
**File:** `day-8.py`

**Components:**
- 1x DS18B20 temperature sensor
- 1x 4.7KΩ resistor
- 3x Jumper wires

**Pin Mapping:**
- GPIO 16: DS18B20 (1-wire protocol)

**Features:**
- Non-blocking temperature reading (2-stage)
- Thermal entropy bits
- Temperature influences chaos, mutation, audio
- Warmer = more chaotic
- 8 entropy sources: adds thermal entropy

---

### Day 9: Tilt/Shake Detection
**File:** `day-9.py`

**Components:**
- 1x Ball tilt switch
- 4x Jumper wires

**Pin Mapping:**
- GPIO 21: Tilt switch

**Features:**
- Detects tilting/shaking
- **Energy accumulation system**: builds up with shaking, decays over time
- Mechanical entropy bits
- Shake the Pico → chaos explosion!
- 9 entropy sources: adds mechanical entropy

---

### Day 10: IR Beam Break Detection
**File:** `day-10.py`

**Components:**
- 1x IR beam break sensor (emitter + receiver)
- 1x 10KΩ resistor (external pull-up)
- 4x Jumper wires

**Pin Mapping:**
- GPIO 17: Beam sensor signal (external pull-up)

**Features:**
- Detects beam interruption (0 = broken, 1 = clear)
- **Energy accumulation with decay**: energy spike on break, fades slowly (1% per loop)
- **Recency factor**: high influence for 2 seconds after break
- Break counting for entropy generation
- Wave hand through beam → chaos spike!
- 10 entropy sources: adds detection entropy

---

### Day 11: OLED Display
**File:** `day-11.py`

**Components:**
- 1x 128x32 I2C OLED display (SSD1306)
- 4x Jumper wires

**Pin Mapping:**
- GPIO 6: I2C SDA (data)
- GPIO 7: I2C SCL (clock)

**Features:**
- Real-time chaos parameter display
- Line 1: Temperature and chaos value (X)
- Line 2: Sensor states (Motion, Tilt, Beam) + frequency
- Line 3: Current LED states (R/A/G)
- Updates every loop cycle
- Visual feedback for all system states

---

### Day 12: WS2812 RGB LED Strip (FINAL DAY!)
**File:** `day-12.py`

**Components:**
- 1x WS2812 RGB LED strip (8 LEDs)
- 3x Jumper wires
- Optional: 470Ω resistor for data line protection
- Optional: 1000µF capacitor across power

**Pin Mapping:**
- GPIO 14: WS2812 Data (DIN)

**Features:**
- 8 individually addressable RGB LEDs
- **Color mapping:**
  - Red: Chaos value + Dirac spinor
  - Green: Quantum phase + button input
  - Blue: Temperature + tilt + beam energy
- **Override modes:**
  - Beam broken: All LEDs pure red alarm
  - Motion detected: All LEDs white flash
- Real-time chaos rainbow visualization
- **COMPLETE 12 DAYS BUILD!**

---

### Day 13: Red Diffused LED (Let It Glow!)
**File:** `day-13.py`

**Components:**
- 1x 15mm diffused red LED
- 1x 100Ω resistor
- 2x Male-to-male jumper wires

**Pin Mapping:**
- GPIO 0: Red LED output (+ 100Ω resistor)

**Features:**
- **Chaos intensity indicator** - lights when chaos is high
- Combines chaos value (x) + beam energy + tilt energy
- Threshold trigger at 50% intensity
- Visual feedback for system activity level
- Complements existing 3-LED chaos indicators

---

### Day 14: Two Additional Buttons (Let It Glow)
**File:** `day-14.py`

**Components:**
- 2x 12mm square tactile buttons
- 2x Colorful button caps
- 5x Male-to-male jumper wires

**Pin Mapping:**
- GPIO 1: Button 4 (PULL_UP)
- GPIO 2: Button 5 (PULL_UP)

**Features:**
- **5 total buttons** for chaos control (3 original + 2 new)
- Combined button pressure calculation
- New buttons influence:
  - Red LED chaos intensity indicator
  - Future chaos parameters
- Colorful caps for easy identification
- Active LOW with internal pull-up

---

## Complete Pin Mapping Reference

| GPIO | Component | Day | Type | Notes |
|------|-----------|-----|------|-------|
| 0 | Red Diffused LED | 13 | Digital OUT | + 100Ω resistor |
| 1 | Button 4 | 14 | Digital IN | PULL_UP |
| 2 | Button 5 | 14 | Digital IN | PULL_UP |
| 3 | Button 3 | 3 | Digital IN | PULL_UP |
| 6 | OLED SDA | 11 | I2C Data | SSD1306 128x32 |
| 7 | OLED SCL | 11 | I2C Clock | 400kHz |
| 8 | Button 2 | 3 | Digital IN | PULL_UP |
| 13 | Button 1 | 3 | Digital IN | PULL_UP |
| 14 | WS2812 Data | 12 | Digital OUT | RGB LED strip |
| 15 | Piezo | 5 | PWM OUT | 200-4000 Hz |
| 16 | Temperature | 8 | 1-Wire | DS18B20 |
| 17 | Beam Break | 10 | Digital IN | External pull-up |
| 18 | Red LED | 2 | Digital OUT | + 330Ω resistor |
| 19 | Amber LED | 2 | Digital OUT | + 330Ω resistor |
| 20 | Green LED | 2 | Digital OUT | + 330Ω resistor |
| 21 | Tilt Switch | 9 | Digital IN | PULL_UP |
| 22 | PIR Motion | 7 | Digital IN | Active HIGH |
| 25 | Built-in LED | 1 | Digital OUT | On-board |
| 26 (ADC0) | Potentiometer | 4 | Analog IN | 0-3.3V |
| 27 (ADC1) | Noise Source | 2 | Analog IN | Floating |
| 28 (ADC2) | Phototransistor | 6 | Analog IN | 0-3.3V |

---

## Chaos System Architecture

### Entropy Sources (10 total)
1. **Environmental Noise** (ADC27) - Random electrical noise
2. **Logistic Map Chaos** - Deterministic chaos function
3. **Mutation State** - Evolving bitfield memory
4. **Quantum Oscillator** - Rabi-like phase oscillations
5. **Dirac Spinor** - 4-component toy spinor modulation
6. **Button Input** - User interaction (Day 3+)
7. **Optical Randomness** - Light sensor (Day 6+)
8. **Thermal Entropy** - Temperature variations (Day 8+)
9. **Mechanical Entropy** - Tilt/shake detection (Day 9+)
10. **Detection Entropy** - Beam break events (Day 10+)

### Fusion Method
All entropy bits are combined using XOR, with probabilistic bit-flipping influenced by:
- Spinor components
- Button pressure
- Potentiometer position
- PIR motion state
- Temperature
- Tilt energy

### Output Channels
- **3 LEDs**: Visual chaos patterns
- **Piezo**: Audible frequency chaos (200-4000 Hz)
- **Serial**: Debug output (tilt events, PIR triggers, temp readings)

---

## Speed Control (Potentiometer)

The potentiometer provides extreme speed control:

- **Pot at 0 (minimum)**: 500ms delay → ~2 loops/second (VERY SLOW)
- **Pot at 1 (maximum)**: 0.1ms delay → ~10,000 loops/second (EXTREMELY FAST)
- **Speed range**: 5000x difference!

Turn left for slow, visible patterns. Turn right for blur of chaos.

---

## Special Features

### Tilt Energy Accumulation (Day 9)
- Each tilt/shake adds 0.3 energy (caps at 1.0)
- Energy decays 2% per loop
- Creates "burst and decay" chaos patterns
- Shake violently → explosive chaos that calms down

### Non-Blocking Temperature (Day 8)
- 2-stage reading prevents 750ms blocking delay
- Temperature read every 2 seconds
- Continuous influence using cached value

### PIR Burst Mode (Day 7)
- Motion triggers 150ms "flinch" response
- All LEDs flash, piezo screeches
- Immediate reaction to movement

---

## Getting Started

1. **Day 1**: Upload `day-1.py` to test basic hardware
2. **Day 2-9**: Add components progressively, upload corresponding file
3. **Run**: Each day's file is standalone and includes all previous components
4. **Experiment**: Adjust pot for speed, shake for chaos, wave hand for PIR trigger

---

## Wiring Tips

- Use color-coded jumper wires for easy debugging
- Keep LED resistors close to the LEDs
- Temperature sensor needs 4.7KΩ pull-up between data and 3.3V
- Phototransistor needs 10KΩ pull-down resistor
- Tilt switch works best with PULL_UP configuration

---

## Code Features

- Manual shuffle implementation (MicroPython compatible)
- No external dependencies beyond `machine` module
- All files syntax-checked and optimized
- Comments explain each chaos component
- Debug prints for key events (PIR, tilt, temperature)

---

## 12 Days Complete!

**All 12 days have been implemented!** The chaos system is now complete with:
- 10 entropy sources feeding the chaos engine
- Real-time OLED visualization
- RGB LED strip showing chaos colors
- Full sensor suite (buttons, pot, temp, motion, tilt, beam)
- Audio chaos output via piezo
- Progressive build from simple blink to full multi-sensor system

---

## Troubleshooting

**LEDs not lighting:**
- Check resistor connections (330Ω)
- Verify GPIO pins (18, 19, 20)
- Test with day-1.py first

**Pot not controlling speed:**
- Verify ADC26 connection
- Check pot is 10KΩ
- Run day-4.py or later

**Temperature not reading:**
- Ensure 4.7KΩ resistor between data and 3.3V
- Check GPIO 16 connection
- Wait 2 seconds for first reading

**Tilt not responding:**
- Verify GPIO 21 connection
- Check PULL_UP is working
- Tilt should print "TILT CHANGE!" messages

**Piezo too loud/quiet:**
- Adjust duty cycle in code (1000-3000 range)
- Check GPIO 15 PWM connection

---

## License

Open source - use for educational and maker projects!

## Credits

12 Days of Codemas Advent Calendar Project
MicroPython chaos system implementation
