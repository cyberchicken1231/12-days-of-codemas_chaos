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

### Day 15: 5-Segment Bar Graph Display (Let It Glow)
**File:** `day-15.py`

**Components:**
- 1x 5-segment bar graph LED display
- 1x Network resistor (typically 220Ω or 330Ω)
- 6x Male-to-male jumper wires

**Pin Mapping:**
- GPIO 4: Segment 1 (lowest/first LED)
- GPIO 5: Segment 2
- GPIO 9: Segment 3 (middle)
- GPIO 10: Segment 4
- GPIO 11: Segment 5 (highest/last LED)

**Features:**
- **Visual chaos intensity meter** - bar fills based on chaos level
- 0-5 segments light up proportionally (0.0-1.0 range)
- Displays same chaos_intensity as red LED threshold
- Bar fills from bottom to top
- Real-time analog-style visualization
- Network resistor connects all cathodes to GND

---

### Day 16: Dual-Pico GPIO Expansion - 5-Way DIP Switch (Let It Glow)
**Files:** `day-16.py` (main), `gpio_expander_slave.py` (second Pico)

**Components:**
- 1x Additional Raspberry Pi Pico H (GPIO expander)
- 1x 5-way DIP switch
- 3x Male-to-male jumper wires (UART connection)
- 5x Jumper wires for DIP switch

**Main Pico Pin Mapping:**
- GPIO 12: UART TX to slave Pico (Physical pin 16)
- GPIO 13: UART RX from slave Pico (Physical pin 17)
- GND: Common ground with slave Pico

**⚠️ Hardware Change:** Original 3 buttons from Day 3 (GP3, GP8, GP13) removed to free GPIO pins for UART. System now uses only 2 buttons (Day 14: GP1, GP2).

**Slave Pico Pin Mapping:**
- GPIO 0: UART TX to main Pico (Physical pin 1)
- GPIO 1: UART RX from main Pico (Physical pin 2)
- GPIO 2-6: DIP switch inputs (Physical pins 4-9)
- GND: Common ground with main Pico

**Features:**
- **GPIO expansion** - Second Pico provides 16 additional GPIOs via UART
- **5-way DIP switch** - Manual configuration switches
- **UART communication** - Simple request-response protocol at 9600 baud
- **DIP switch entropy** - Switch state adds 11th chaos source
- **Real-time polling** - Main Pico requests switch state each loop
- **Failsafe design** - Returns all-OFF state on timeout
- **OLED display** - Shows DIP switch count (0-5) on screen

**Protocol:**
- Main sends 'R' (Read request)
- Slave responds with 1 byte (5 bits = switch states)
- 100ms timeout for reliability

**Wiring:**
```
Main Pico          Slave Pico
---------          ----------
GP12 (TX) -------> GP1 (RX)
GP13 (RX) <------- GP0 (TX)
GND <------------> GND
```

**DIP Switch Wiring (on Slave Pico):**
- Each switch connects GPIO to GND when ON
- Internal pull-ups: HIGH = OFF, LOW = ON
- Compact 5-switch package

---

### Day 17: 2x Addressable RGB LEDs (Let It Glow)
**File:** `day-17.py`

**Components:**
- 2x 10mm WS2812 Addressable RGB LEDs
- 6x Jumper wires (3 per LED)
- Optional: 2x 470Ω resistors for data line protection

**Main Pico Pin Mapping:**
- GPIO 3: RGB LED 0 data line (Physical pin 5) - Chaos rainbow

**Slave Pico Pin Mapping:**
- GPIO 7: RGB LED 1 data line (Physical pin 10) - Activity indicator

**Power (both LEDs):**
- VCC: 3.3V or 5V
- GND: Ground

**Features:**
- **Dual RGB indicators** - 2 fully independent addressable LEDs on separate Picos
- **LED 0 (Main GP3): Chaos Rainbow** - Cycles through full color spectrum based on logistic map value (x)
- **LED 1 (Slave GP7): Activity Indicator** - Controlled via UART, shows sensor activity levels:
  - **High activity (>70%)**: Red-Orange (tilt/beam/DIP active)
  - **Medium activity (40-70%)**: Yellow-Green
  - **Low activity (<40%)**: Blue-Purple (calm)
- **HSV color space** - Smooth hue transitions for LED 0
- **UART color commands** - Main Pico sends RGB values to slave for LED 1
- **Real-time updates** - Colors change every loop iteration
- **10mm diffused lens** - Wide viewing angle
- **Distributed control** - LED 0 on main Pico, LED 1 on slave Pico

**Color Modes:**
- **Chaos LED** (LED 0): Hue = `x * 360°`, full saturation, 50% brightness
- **Activity LED** (LED 1): Blends tilt energy, beam break energy, and DIP switch value

**Protocol:**
- Main sends `'C' + R + G + B` (4 bytes) to set LED 1 color
- Slave responds with `'K'` confirmation

**Wiring:**
```
Main Pico GP3 (pin 5) --> RGB LED 0 DIN
Slave Pico GP7 (pin 10) --> RGB LED 1 DIN

Power both LEDs:
3.3V or 5V --> VCC (both)
GND --> GND (both)
```

**Notes:**
- LED 1 on slave Pico eliminates GPIO constraints on main Pico
- UART protocol extended for RGB color commands
- Optional 470Ω resistor on each data line for protection
- 100µF capacitor across power supply recommended for stability
- WS2812 LEDs work at both 3.3V and 5V logic levels

---

### Day 18: 12-LED RGB Ring (Let It Glow)
**File:** `day-18.py`

**Components:**
- 1x 12-LED WS2812 RGB Ring (NeoPixel compatible)
- 3x Jumper wires
- Optional: 470Ω resistor for data line protection

**Slave Pico Pin Mapping:**
- GPIO 8: RGB Ring data line (Physical pin 12) - 12 LEDs

**Power:**
- VCC: 3.3V or 5V
- GND: Ground

**Features:**
- **12-LED rotating rainbow** - Spinning color wheel pattern
- **Chaos-driven rotation** - Ring spins based on logistic map value
- **Dynamic brightness** - Varies with button pressure and Dirac spinor energy
- **UART control** - Main Pico sends pattern commands to slave
- **Smooth animation** - 30° hue spacing between adjacent LEDs
- **Real-time updates** - Pattern changes every loop
- **Distributed processing** - Ring rendering on slave Pico

**Pattern Algorithm:**
- Each LED offset by 30° hue (360° / 12 LEDs)
- Rotation driven by chaos value + spinor component
- Brightness = 0.3 base + buttons (0.3 max) + spinor (0.2 max)

**Protocol:**
- Main sends `'N' + hue_offset (0-255) + brightness (0-255)` (3 bytes)
- Slave updates 12-LED ring and responds with `'K'`

**Wiring:**
```
Slave Pico GP8 (pin 12) --> Ring DIN
5V --> Ring VCC (recommended for brightness)
GND --> Ring GND
```

**Notes:**
- Ring on slave Pico saves main Pico bandwidth
- Only 3 bytes sent per update vs 36 bytes for full RGB control
- Pattern generated locally on slave for efficiency
- 5V power recommended for full brightness
- Optional 470Ω + 1000µF capacitor for power stability

---

### Day 20: DHT20 Temperature & Humidity Sensor (Let It Glow)
**File:** `day-20.py`

**Components:**
- 1x DHT20 I2C Temperature & Humidity Sensor
- 4x Jumper wires (VCC, GND, SDA, SCL)

**Slave Pico Pin Mapping:**
- GPIO 14: I2C1 SDA (Physical pin 19) - Data line
- GPIO 15: I2C1 SCL (Physical pin 20) - Clock line

**Sensor Specifications:**
- I2C Address: 0x38
- Temperature Range: -40°C to +80°C (±0.5°C accuracy)
- Humidity Range: 0% to 100% RH (±3% accuracy)
- Operating Voltage: 3.3V or 5V
- Response Time: <8 seconds

**Features:**
- **Environmental sensing** - Measures ambient temperature and humidity
- **Chaos modulation** - Temperature affects logistic map parameter 'r'
- **Display modulation** - Humidity affects bar graph brightness
- **UART protocol** - Main Pico requests readings from slave
- **Real-time feedback** - Sensor readings printed to serial
- **Auto-calibration** - Sensor self-calibrates on initialization
- **Error handling** - Graceful degradation if sensor fails

**Integration:**
- **Temperature influence**: Warmer temps = faster/more chaotic behavior
  - Formula: `temp_influence_dht = ((temp_c - 20) / 10.0) * 0.1`
  - Nominal: 20-30°C → -0.1 to +0.1 influence on 'r' parameter
- **Humidity influence**: Higher humidity = brighter/more active display
  - Formula: `humidity_influence = (humidity - 50) / 50.0`
  - Nominal: 40-60% RH → modulates bar graph level ±15%

**Protocol:**
- Main sends `'T'` (1 byte)
- Slave reads DHT20 and sends 4 bytes: `temp_int, temp_frac, hum_int, hum_frac`
- Example: `23, 45, 55, 32` = 23.45°C, 55.32% RH
- On error: Slave sends `255, 255, 255, 255`

**Wiring:**
```
Slave Pico GP14 (pin 19) --> DHT20 SDA
Slave Pico GP15 (pin 20) --> DHT20 SCL
3.3V --> DHT20 VCC
GND --> DHT20 GND
```

**Notes:**
- DHT20 is digital I2C sensor (more accurate than DHT11/DHT22)
- No pull-up resistors needed (built into sensor module)
- Measurement takes ~80ms (non-blocking on slave)
- Use main Pico's DS18B20 for comparison/redundancy
- Both temp sensors contribute to chaos entropy
- I2C bus runs at 400kHz for fast reads

---

### Day 21: LED String Lights (Let It Glow)
**File:** `day-21.py`

**Components:**
- 1x Addressable LED String Lights (~15 LEDs, WS2812/NeoPixel compatible)
- 3x Jumper wires (Data, VCC, GND)

**Slave Pico Pin Mapping:**
- GPIO 9: LED String data line (Physical pin 12)

**Power:**
- VCC: 5V recommended (can use 3.3V at lower brightness)
- GND: Ground

**Features:**
- **4 animated patterns** - Chase, Twinkle, Wave, and Chaos modes
- **Chaos-driven pattern selection** - Logistic map value determines active pattern
- **Spinor-modulated speed** - Animation speed varies with Dirac spinor energy
- **Humidity-modulated brightness** - Higher humidity = brighter lights
- **Autonomous animation** - Patterns run continuously on slave Pico
- **UART control** - Main Pico sends pattern parameters, slave handles rendering

**Pattern Modes:**
- **0: Chase (Running Lights)** - Comet effect with warm white trailing fade
- **1: Twinkle (Random Sparkles)** - Cool white random twinkling stars
- **2: Wave (Smooth Pulsing)** - Warm amber sine wave across string
- **3: Chaos (Pseudo-Random Colors)** - Deterministic rainbow chaos pattern

**Pattern Algorithm:**
- Pattern mode = `int(chaos_val * 4) % 4` (0-3)
- Speed = `0.3 + spinor_energy * 0.7` (0-1)
- Brightness = `0.5 + (humidity / 100) * 0.3` (50-80%)
- Update interval = `50-200ms` (faster at high speed)

**Protocol:**
- Main sends `'L' + pattern_mode (0-255) + speed (0-255) + brightness (0-255)` (4 bytes)
- Slave updates pattern parameters and responds with `'K'`
- Animation continues autonomously between updates

**Wiring:**
```
Slave Pico GP9 (pin 12) --> String DIN
5V --> String VCC (recommended for full brightness)
GND --> String GND
```

**Notes:**
- Festive string light effect perfect for holiday chaos
- Pattern runs independently - doesn't block UART communication
- Speed adaptive: slow chaos = slow patterns, fast chaos = rapid animations
- Each pattern has unique color temperature (warm/cool/amber/rainbow)
- Can be extended to 25+ LEDs by changing `NUM_STRING_LEDS`
- Patterns generated locally on slave for efficiency

---

## Complete Pin Mapping Reference

| GPIO | Component | Day | Type | Notes |
|------|-----------|-----|------|-------|
| 0 | Red Diffused LED | 13 | Digital OUT | + 100Ω resistor |
| 1 | Button 4 | 14 | Digital IN | PULL_UP |
| 2 | Button 5 | 14 | Digital IN | PULL_UP |
| 3 | RGB LED 0 Data | 17 | Digital OUT | WS2812 - Chaos rainbow |
| 4 | Bar Graph Seg 1 | 15 | Digital OUT | Network resistor |
| 5 | Bar Graph Seg 2 | 15 | Digital OUT | Network resistor |
| 6 | OLED SDA | 11 | I2C Data | SSD1306 128x32 |
| 7 | OLED SCL | 11 | I2C Clock | 400kHz |
| 8 | ~~Button 2~~ (removed) | ~~3~~ | - | Freed for expansion |
| 9 | Bar Graph Seg 3 | 15 | Digital OUT | Network resistor |
| 10 | Bar Graph Seg 4 | 15 | Digital OUT | Network resistor |
| 11 | Bar Graph Seg 5 | 15 | Digital OUT | Network resistor |
| 12 | UART TX (to slave) | 16 | UART TX | 9600 baud to slave Pico |
| 13 | UART RX (from slave) | 16 | UART RX | 9600 baud from slave Pico |
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

### Entropy Sources (11 total)
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
11. **Configuration Entropy** - DIP switch settings (Day 16+)

### Fusion Method
All entropy bits are combined using XOR, with probabilistic bit-flipping influenced by:
- Spinor components
- Button pressure
- Potentiometer position
- PIR motion state
- Temperature
- Tilt energy
- Beam break energy
- DIP switch configuration

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
