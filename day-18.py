# fused_chaos_with_quantum_and_dirac_influence.py (Day 18 - Let It Glow!)
# MicroPython on RP2040 (Pico) - Main Controller
#
# Day 18: 12-LED RGB Ring
# Day 17: 2x Addressable RGB LEDs
# Day 16: Dual-Pico GPIO Expansion via UART
# - Main Pico (this one) runs the chaos system
# - Second Pico acts as GPIO expander for 5-way DIP switch
# - Communication via UART (9600 baud)
#
# UART Wiring:
# - Main GP12 (TX) --> Slave GP1 (RX)  [Physical pin 16 -> pin 2]
# - Main GP13 (RX) --> Slave GP0 (TX)  [Physical pin 17 -> pin 1]
# - GND --> GND (common ground required)
#
# Note: Original 3 buttons (Day 3) removed to free GPIO pins

from machine import Pin, ADC, I2C, UART
import time
import random
import math
import onewire
import ds18x20
from ssd1306 import SSD1306_I2C
import neopixel

# -------------------------
# Day 16: UART to GPIO Expander Pico
# -------------------------
# Initialize UART0 for communication with slave Pico
# Using GP12 (TX) and GP13 (RX) - original buttons removed
uart_expander = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13))

def read_dip_switch():
    """Read DIP switch state from slave Pico via UART
    Returns list of 5 values: [sw1, sw2, sw3, sw4, sw5]
    Each value is 0 (OFF) or 1 (ON)
    """
    try:
        # Send read request
        uart_expander.write(b'R')

        # Wait for response (with timeout)
        start = time.ticks_ms()
        while not uart_expander.any():
            if time.ticks_diff(time.ticks_ms(), start) > 100:  # 100ms timeout
                return [0, 0, 0, 0, 0]  # Return all OFF on timeout
            time.sleep_ms(1)

        # Read response byte
        response = uart_expander.read(1)
        if response:
            state_byte = response[0]
            # Convert byte to list of 5 bits
            return [
                (state_byte >> 0) & 1,  # Switch 1
                (state_byte >> 1) & 1,  # Switch 2
                (state_byte >> 2) & 1,  # Switch 3
                (state_byte >> 3) & 1,  # Switch 4
                (state_byte >> 4) & 1,  # Switch 5
            ]
    except Exception as e:
        print("UART read error:", e)
        return [0, 0, 0, 0, 0]

    return [0, 0, 0, 0, 0]

print("UART GPIO Expander initialized on UART0 (GP12/13)")

# -------------------------
# Day 6: Convert light to entropy bits
# -------------------------
def get_photo_noise():
    raw = photo.read_u16()          # 0–65535
    bright = raw / 65535.0          # normalize to 0.0–1.0

    # As light increases → probability of flipping increases
    flip_strength = bright * 0.25   # up to 25%

    bits = []
    for i in range(3):
        b = random.getrandbits(1)

        # light-driven flip
        if random.random() < flip_strength:
            b ^= 1

        # tiny extra chaos spur
        if random.random() < (bright * 0.08):
            b ^= 1

        bits.append(b)

    return bits, bright

# -------------------------
# Dirac toy spinor (4 components)
# -------------------------
def fake_dirac_spinor(t, buttons):
    b0, b1, b2 = buttons

    phase = (
        t
        + 0.8 * b0 * math.sin(3 * t)
        + 0.5 * b1 * math.sin(9 * t)
    )

    amp = 1.0 + 0.4 * b2 * math.sin(5 * t)

    return [
        amp * math.sin(phase),
        amp * math.cos(phase),
        amp * math.sin(2 * phase),
        amp * math.cos(2 * phase),
    ]

# -------------------------
# LEDs
# -------------------------
red = Pin(18, Pin.OUT)
amber = Pin(19, Pin.OUT)
green = Pin(20, Pin.OUT)

# -------------------------
# Day 13: Red Diffused LED (Let It Glow)
# -------------------------
red_led = Pin(0, Pin.OUT)  # GPIO 0 (physical pin 1) - 15mm diffused red LED

# -------------------------
# Day 15: 5-Segment Bar Graph Display (Let It Glow)
# -------------------------
# 5 segments for chaos level visualization (low to high)
bar_segments = [
    Pin(4, Pin.OUT),   # Segment 1 (lowest)  - GPIO 4
    Pin(5, Pin.OUT),   # Segment 2           - GPIO 5
    Pin(9, Pin.OUT),   # Segment 3 (middle)  - GPIO 9
    Pin(10, Pin.OUT),  # Segment 4           - GPIO 10
    Pin(11, Pin.OUT),  # Segment 5 (highest) - GPIO 11
]

def update_bar_graph(level):
    """Update bar graph based on level (0.0 to 1.0)"""
    # Convert level to number of segments (0-5)
    num_segments = int(level * 5)
    num_segments = max(0, min(5, num_segments))  # Clamp to 0-5

    # Light up segments from bottom to top
    for i in range(5):
        bar_segments[i].value(1 if i < num_segments else 0)

# -------------------------
# Day 5: Piezo PWM
# -------------------------
from machine import PWM
piezo = PWM(Pin(15))
piezo.freq(440)      # default tone, overwritten every loop
piezo.duty_u16(0)    # start silent

# -------------------------
# Environmental noise (floating ADC)
# -------------------------
noise = ADC(27)

# -------------------------
# Day 3: Buttons (REMOVED - GPIO freed for UART)
# -------------------------
# Original 3 buttons removed to free GP3, GP8, GP13 for other uses
# GP13 now used for UART RX to slave Pico

def read_buttons():
    """Return dummy button values (all OFF) since original buttons removed"""
    return [0, 0, 0]

# -------------------------
# Day 14: Two More Buttons (Let It Glow)
# -------------------------
button4 = Pin(1, Pin.IN, Pin.PULL_UP)  # GPIO 1 (physical pin 2)
button5 = Pin(2, Pin.IN, Pin.PULL_UP)  # GPIO 2 (physical pin 4)

def read_new_buttons():
    return [
        0 if button4.value() else 1,  # Button 4
        0 if button5.value() else 1,  # Button 5
    ]

# -------------------------
# Day 4: Potentiometer (ADC26)
# -------------------------
pot = ADC(26)
# -------------------------
# Day 6: Phototransistor (ADC28)
# -------------------------
photo = ADC(28)
# -------------------------
# Day 7: PIR Motion Sensor
# -------------------------
pir = Pin(22, Pin.IN)  # Motion = 1, No motion = 0

# -------------------------
# Burst/edge state for PIR immediate reaction
# -------------------------
last_pir = 0
pir_burst_until = 0   # ticks_ms timestamp until which override remains active

# -------------------------
# Day 8: DS18B20 Temperature Sensor
# -------------------------
# Initialize 1-wire bus on GPIO 16 (physical pin 21)
temp_pin = Pin(16)
temp_sensor = ds18x20.DS18X20(onewire.OneWire(temp_pin))

# Scan for DS18B20 devices
temp_roms = temp_sensor.scan()
print("Found DS18B20 devices:", temp_roms)

# Temperature tracking
last_temp_read = 0
current_temp = 20.0  # default starting temperature in Celsius
TEMP_READ_INTERVAL = 2000  # read every 2 seconds to avoid blocking (sensor conversion takes ~750ms)
temp_conversion_started = False
temp_conversion_time = 0

# -------------------------
# Day 9: Ball Tilt Switch
# -------------------------
tilt = Pin(21, Pin.IN, Pin.PULL_UP)  # GPIO 21 (physical pin 27)
# Tilt switch: normally closed when level, opens when tilted
# With PULL_UP: 0 = tilted, 1 = level

last_tilt = 1
tilt_change_count = 0  # Track how many times it's been tilted
tilt_energy = 0.0      # Accumulated tilt energy that decays over time

# -------------------------
# Day 10: IR Beam Break Sensor
# -------------------------
beam = Pin(17, Pin.IN)  # GPIO 17 (physical pin 22) - external pull-up
# Beam break: 0 = beam broken (object in path), 1 = beam clear

last_beam = 1
beam_break_count = 0   # Total number of breaks
beam_break_energy = 0.0  # Accumulated break energy (decays)
last_beam_time = 0     # Time of last beam break

# -------------------------
# Day 11: OLED Display (128x32 I2C)
# -------------------------
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)  # I2C1 on GPIO 6/7
oled = SSD1306_I2C(128, 32, i2c)  # 128x32 pixel display

# Waveform buffer for display (scrolling graph)
wave_buffer = [0] * 128  # 128 pixels wide
wave_index = 0

# -------------------------
# Day 12: WS2812 RGB LED Strip
# -------------------------
NUM_LEDS = 15  # 15 LEDs on the strip
strip = neopixel.NeoPixel(Pin(14), NUM_LEDS)  # GPIO 14 (physical pin 19)

# -------------------------
# Day 17: 2x Addressable RGB LEDs (Let It Glow)
# -------------------------
# LED 0: Main Pico GP3 (physical pin 5) - Chaos rainbow
# LED 1: Slave Pico GP7 (physical pin 10) - Activity indicator (via UART)
rgb_led0 = neopixel.NeoPixel(Pin(3), 1)  # GPIO 3 (physical pin 5) - Chaos LED on main Pico

# -------------------------
# Day 18: 12-LED RGB Ring (Let It Glow)
# -------------------------
# 12x WS2812 RGB LEDs in ring formation on Slave Pico GP8 (physical pin 12)
# Controlled via UART commands

def send_ring_pattern(hue_offset, brightness):
    """Send ring pattern command to slave Pico
    Protocol: 'N' + hue_offset (0-255) + brightness (0-255)
    """
    try:
        # Convert hue (0-360) to byte (0-255)
        hue_byte = int((hue_offset % 360) / 360.0 * 255)
        brightness_byte = int(brightness * 255)
        brightness_byte = max(0, min(255, brightness_byte))  # Clamp to 0-255

        # Send ring command: 'N' followed by 2 bytes
        uart_expander.write(b'N')
        uart_expander.write(bytes([hue_byte, brightness_byte]))

        # Wait for confirmation (optional, with timeout)
        start = time.ticks_ms()
        while not uart_expander.any():
            if time.ticks_diff(time.ticks_ms(), start) > 50:  # 50ms timeout
                return False
            time.sleep_ms(1)

        # Read confirmation
        if uart_expander.any():
            conf = uart_expander.read(1)
            return conf == b'K'
        return False
    except:
        return False

def update_ring(chaos_val, spinor, button_pressure):
    """Update 12-LED ring with rotating patterns via UART
    Creates spinning rainbow effect driven by chaos
    """
    # Calculate hue offset based on chaos and spinor (0-360 degrees)
    hue_offset = ((chaos_val + spinor[0]) * 360) % 360

    # Brightness varies with button pressure and spinor energy
    brightness = 0.3 + (button_pressure * 0.3) + (abs(spinor[1]) * 0.2)
    brightness = min(1.0, brightness)  # Cap at 1.0

    # Send pattern to slave Pico
    send_ring_pattern(hue_offset, brightness)

def send_rgb_color(r, g, b):
    """Send RGB color command to slave Pico for LED 1
    Protocol: 'C' + R + G + B (4 bytes total)
    """
    try:
        # Send color command: 'C' followed by 3 bytes
        uart_expander.write(b'C')
        uart_expander.write(bytes([r, g, b]))

        # Wait for confirmation (optional, with timeout)
        start = time.ticks_ms()
        while not uart_expander.any():
            if time.ticks_diff(time.ticks_ms(), start) > 50:  # 50ms timeout
                return False
            time.sleep_ms(1)

        # Read confirmation
        if uart_expander.any():
            conf = uart_expander.read(1)
            return conf == b'K'
        return False
    except:
        return False

def update_rgb_leds(chaos_val, dip_val, tilt_energy, beam_energy):
    """Update 2 RGB LEDs based on different entropy sources
    LED 0 (Main GP3): Chaos-driven (logistic map)
    LED 1 (Slave GP7): Sensor-driven (tilt + beam + DIP) via UART
    """
    # LED 0: Chaos rainbow (cycles through spectrum based on x value)
    hue = int(chaos_val * 360)  # 0-360 degrees
    r0, g0, b0 = hsv_to_rgb(hue, 1.0, 0.5)  # Full saturation, 50% brightness
    rgb_led0[0] = (r0, g0, b0)
    rgb_led0.write()

    # LED 1: Sensor mix (warm colors = high activity, cool colors = low activity)
    activity = (tilt_energy * 0.4 + beam_energy * 0.4 + dip_val * 0.2)  # 0-1
    if activity > 0.7:
        # High activity: Red-Orange
        r1, g1, b1 = 200, int(50 + activity * 100), 0
    elif activity > 0.4:
        # Medium activity: Yellow-Green
        r1, g1, b1 = int(150 - activity * 100), 150, 0
    else:
        # Low activity: Blue-Purple
        r1, g1, b1 = int(activity * 100), 0, int(100 + activity * 100)

    # Send color to slave Pico via UART
    send_rgb_color(r1, g1, b1)

def hsv_to_rgb(h, s, v):
    """Convert HSV to RGB
    h: 0-360 (hue degrees)
    s: 0-1 (saturation)
    v: 0-1 (value/brightness)
    Returns: (r, g, b) as 0-255
    """
    h = h % 360
    c = v * s
    x = c * (1 - abs((h / 60) % 2 - 1))
    m = v - c

    if h < 60:
        r, g, b = c, x, 0
    elif h < 120:
        r, g, b = x, c, 0
    elif h < 180:
        r, g, b = 0, c, x
    elif h < 240:
        r, g, b = 0, x, c
    elif h < 300:
        r, g, b = x, 0, c
    else:
        r, g, b = c, 0, x

    return (int((r + m) * 255), int((g + m) * 255), int((b + m) * 255))

# -------------------------
# Chaos & mutation
# -------------------------
x = random.random()
r = 3.9
mut_state = [0, 1, 0]

# -------------------------
# Quantum oscillator
# -------------------------
OMEGA_BASE = 2.0 * math.pi * 2.0
OMEGA = OMEGA_BASE
quantum_phase = 0.0

last_t = time.ticks_ms()

# -------------------------
# Mutation shuffler
# -------------------------
def manual_shuffle(arr):
    for _ in range(2):
        i = random.randint(0, len(arr) - 1)
        j = random.randint(0, len(arr) - 1)
        arr[i], arr[j] = arr[j], arr[i]

# -------------------------
# Main loop
# -------------------------
while True:
    raw_photo = photo.read_u16()
    norm_photo = raw_photo / 65535.0
    print("PHOTO RAW:", raw_photo, "   NORM:", norm_photo)

    now = time.ticks_ms()
    dt_ms = time.ticks_diff(now, last_t)
    if dt_ms < 0:
        dt_ms = 0
    dt = dt_ms / 1000.0
    last_t = now
    t = now / 1000.0

    # -------------------------
    # Read buttons
    # -------------------------
    button_bits = read_buttons()
    button_pressure = sum(button_bits) / 3.0

    # -------------------------
    # Day 14: Read new buttons
    # -------------------------
    new_button_bits = read_new_buttons()
    new_button_pressure = sum(new_button_bits) / 2.0

    # Combined button pressure (all 5 buttons)
    total_button_pressure = (sum(button_bits) + sum(new_button_bits)) / 5.0

    # -------------------------
    # Day 16: Read DIP switch from slave Pico via UART
    # -------------------------
    dip_switches = read_dip_switch()
    dip_switch_value = sum(dip_switches) / 5.0  # Normalized 0.0-1.0
    print("DIP:", dip_switches, "VAL:", dip_switch_value)

    # Convert DIP switch state to entropy bits (use sum as seed)
    dip_sum = sum(dip_switches)
    dip_bits = [
        (dip_sum >> 0) & 1,
        (dip_sum >> 1) & 1,
        (dip_sum >> 2) & 1,
    ]

    # -------------------------
    # Read potentiometer
    # -------------------------
    pot_raw = pot.read_u16()
    pot_norm = pot_raw / 65535.0      # 0.0 → 1.0
    print("POT RAW:", pot_raw, "NORM:", pot_norm)

    # -------------------------
    # Day 6: Phototransistor noise
    # -------------------------
    photo_bits, photo_level = get_photo_noise()

    # -------------------------
    # Day 8: Temperature sensor read (non-blocking with 2-stage approach)
    # -------------------------
    temp_influence = 0.0
    temp_bits = [0, 0, 0]

    # Two-stage temperature reading to avoid blocking
    if temp_roms:
        # Stage 1: Start conversion if interval elapsed and no conversion in progress
        if not temp_conversion_started and time.ticks_diff(now, last_temp_read) >= TEMP_READ_INTERVAL:
            try:
                temp_sensor.convert_temp()
                temp_conversion_started = True
                temp_conversion_time = now
            except Exception as e:
                print("Temp conversion start error:", e)

        # Stage 2: Read temperature after 750ms conversion time
        if temp_conversion_started and time.ticks_diff(now, temp_conversion_time) >= 750:
            try:
                current_temp = temp_sensor.read_temp(temp_roms[0])
                last_temp_read = now
                temp_conversion_started = False

                print("TEMP:", current_temp, "°C")
            except Exception as e:
                print("Temp read error:", e)
                temp_conversion_started = False

    # Calculate temperature influence (use cached current_temp value)
    if current_temp is not None:
        # Normalize temperature to 0.0-1.0 range
        temp_norm = (current_temp - 15.0) / 20.0
        temp_norm = max(0.0, min(1.0, temp_norm))  # clamp to 0-1

        # Temperature influence on chaos (warmer = more chaotic)
        temp_influence = temp_norm * 0.3

        # Convert temperature to entropy bits
        temp_frac = (current_temp * 100) % 8  # 0-7
        temp_bits = [
            (int(temp_frac) >> 0) & 1,
            (int(temp_frac) >> 1) & 1,
            (int(temp_frac) >> 2) & 1,
        ]

    # Day 7: PIR read
    pir_state = pir.value()  # 1 = motion detected
    # detect rising edge to trigger a short immediate burst
    if pir_state and not last_pir:
        # burst for 150 ms (adjust as you like)
        pir_burst_until = time.ticks_add(now, 150)
        print("PIR RISING EDGE: burst until", pir_burst_until)
    last_pir = pir_state

    # extra jitter from PIR presence (used in flip chance)
    pir_jitter = 0.15 if pir_state == 1 else 0.0

    # speed multiplier (0.3× to ~2.3×)
    speed_scale = 0.3 + (pot_norm * 2.0)
    print("PIR:", pir_state)

    # -------------------------
    # Day 9: Tilt switch read and energy accumulation
    # -------------------------
    tilt_state = tilt.value()  # 0 = tilted, 1 = level

    # Detect tilt state changes (shaking/tilting adds energy)
    if tilt_state != last_tilt:
        tilt_change_count += 1
        tilt_energy = min(1.0, tilt_energy + 0.3)  # Add energy on each tilt change, cap at 1.0
        print("TILT CHANGE! Count:", tilt_change_count, "Energy:", tilt_energy)
    last_tilt = tilt_state

    # Tilt energy decays over time (simulates calming down after shaking)
    tilt_energy *= 0.98  # 2% decay per loop

    # Tilt influence: combination of current state and accumulated energy
    tilt_influence = (1 - tilt_state) * 0.3 + tilt_energy * 0.5  # Max 0.8 total

    # Convert tilt to entropy bits (use change count for randomness)
    tilt_bits = [
        (tilt_change_count >> 0) & 1,
        (tilt_change_count >> 1) & 1,
        (tilt_change_count >> 2) & 1,
    ]

    # -------------------------
    # Day 10: Beam break detection
    # -------------------------
    beam_state = beam.value()  # 0 = broken, 1 = clear

    # Count beam breaks (edge detection for counting only)
    if beam_state == 0 and last_beam == 1:
        beam_break_count += 1
        beam_break_energy = min(1.0, beam_break_energy + 0.4)  # Big energy spike on break
        last_beam_time = now
        print("BEAM BROKEN! Count:", beam_break_count, "Energy:", beam_break_energy)
    last_beam = beam_state

    # Beam energy decays slower than tilt (1% per loop)
    beam_break_energy *= 0.99

    # Recent break creates high influence (fades over ~2 seconds)
    time_since_break = time.ticks_diff(now, last_beam_time)
    recency_factor = max(0, 1.0 - (time_since_break / 2000.0))  # 0-1 over 2 seconds

    # Beam influence: current break state + accumulated energy + recency
    beam_influence = (1 - beam_state) * 0.4 + beam_break_energy * 0.3 + recency_factor * 0.3

    # Convert beam to entropy bits (use break count)
    beam_bits = [
        (beam_break_count >> 0) & 1,
        (beam_break_count >> 1) & 1,
        (beam_break_count >> 2) & 1,
    ]

    # extra bit-flip energy - AGGRESSIVE pot response + tilt + beam + DIP
    pot_flip = pot_norm * 0.6 + tilt_influence * 0.3 + beam_influence * 0.25 + dip_switch_value * 0.2

    # -------------------------
    # Dirac spinor influence
    # -------------------------
    spin = fake_dirac_spinor(t, button_bits)
    # Normalize spinor components from [-1, 1] to [0, 1] for probability calculations
    s0 = (spin[0] + 1) / 2
    s1 = (spin[1] + 1) / 2
    s2 = (spin[2] + 1) / 2
    s3 = (spin[3] + 1) / 2

    # Convert spinor components to probabilistic bits (higher spinor = more likely 1)
    spin_bits = [
        1 if s0 > random.random() else 0,
        1 if s1 > random.random() else 0,
        1 if s2 > random.random() else 0,
    ]

    # -------------------------
    # ADC noise bits
    # -------------------------
    n = noise.read_u16()
    noise_bits = [(n >> i) & 1 for i in range(3)]

    # -------------------------
    # Logistic map chaos
    # -------------------------
    # Adjust r parameter based on spinor, button, temp, pot, tilt, BEAM, and DIP
    r = 3.5 + s0 * 0.49 + button_pressure * 0.1 + temp_influence * 0.15 + pot_norm * 0.2 + tilt_influence * 0.15 + beam_influence * 0.18 + dip_switch_value * 0.12
    x = r * x * (1 - x)
    # Reset if x escapes valid range
    if not (0 < x < 1):
        x = random.random()

    # Use chaos to select which LED to turn OFF (creates chaotic pattern)
    chaos_index = int(x * 3)
    chaos_bits = [1, 1, 1]
    chaos_bits[chaos_index] = 0  # Turn off one LED based on chaos

    # -------------------------
    # Mutation
    # -------------------------
    # Temperature, POT, TILT, BEAM, and DIP add various noise sources to mutation
    mutate_chance = 0.05 + s2 * 0.35 + button_pressure * 0.2 + temp_influence * 0.15 + pot_norm * 0.25 + tilt_influence * 0.2 + beam_influence * 0.18 + dip_switch_value * 0.15
    if random.random() < mutate_chance:
        idx = random.randint(0, 2)
        mut_state[idx] = random.randint(0, 1)

    if random.random() < (0.15 + s2 * 0.2 + button_pressure * 0.2):
        manual_shuffle(mut_state)

    # -------------------------
    # Quantum oscillator
    # -------------------------
    # POT, TILT, BEAM, and DIP add aggressive frequency modulation
    OMEGA = OMEGA_BASE * (1.0 + (s1 - 0.5) * 0.6 + button_pressure * 0.3 + pot_norm * 0.5 + tilt_influence * 0.4 + beam_influence * 0.35 + dip_switch_value * 0.3)
    if OMEGA < 0.1:
        OMEGA = 0.1

    quantum_phase += OMEGA * dt
    # Prevent phase overflow while maintaining periodicity
    if quantum_phase > 1e6:
        quantum_phase %= 2 * math.pi

    # Generate quantum bits using phase-shifted oscillators
    q_bits = []
    for i in range(3):
        off = i * 0.7  # Phase offset for each LED
        p = math.sin(0.5 * (quantum_phase + off)) ** 2  # Probability from oscillator
        bias = 0.05 * spin_bits[i] + 0.1 * button_bits[i]  # Input influence
        q_bits.append(1 if (p + bias > random.random()) else 0)

    # -------------------------
    # Fusion - XOR all sources together for maximum entropy
    # -------------------------
    final = []
    for i in range(3):
        # Combine all bit sources using XOR (each source influences final state)
        mixed = (
            noise_bits[i]      # Environmental randomness
            ^ chaos_bits[i]    # Deterministic chaos
            ^ mut_state[i]     # Mutation memory
            ^ q_bits[i]        # Quantum oscillator
            ^ spin_bits[i]     # Dirac spinor
            ^ button_bits[i]   # User input
            ^ photo_bits[i]    # Day 6: optical randomness
            ^ temp_bits[i]     # Day 8: thermal entropy
            ^ tilt_bits[i]     # Day 9: mechanical entropy (shaking/tilting)
            ^ beam_bits[i]     # Day 10: beam break detection
            ^ dip_bits[i]      # Day 16: DIP switch configuration
        )

        # Add probabilistic bit flip for extra unpredictability
        # Temperature, TILT, BEAM, and DIP add thermal/mechanical/detection/configuration noise
        flip_chance = 0.02 + 0.06 * s3 + 0.05 * button_pressure + pot_flip + pir_jitter + temp_influence * 0.1 + tilt_influence * 0.15 + beam_influence * 0.12 + dip_switch_value * 0.1
        if random.random() < flip_chance:
            mixed ^= 1

        final.append(mixed)

    # -------------------------
    # Output LEDs (normal)
    # -------------------------
    red.value(final[0] ^ pir_state)
    amber.value(final[1] ^ pir_state)
    green.value(final[2] ^ pir_state)

    # -------------------------
    # Day 13: Red LED output (chaos intensity indicator)
    # -------------------------
    # Lights up when chaos is high - combines multiple entropy sources
    # Day 14: Now also responds to new buttons
    # Day 16: DIP switch adds to chaos intensity
    chaos_intensity = x * 0.4 + beam_break_energy * 0.2 + tilt_energy * 0.2 + new_button_pressure * 0.2 + dip_switch_value * 0.2
    red_led.value(1 if chaos_intensity > 0.5 else 0)

    # -------------------------
    # Day 15: Update 5-segment bar graph
    # -------------------------
    # Display chaos value as bar graph (0-5 segments)
    # Use raw chaos value x (0-1) for better visualization
    update_bar_graph(x)

    # -------------------------
    # Day 5: Chaos audio output
    # -------------------------
    # Pick a frequency based on the fusion of chaos components.
    # Piezo reacts to: logistic map + spinor + q-state + buttons + temp + tilt + beam + DIP
    audio_base = 200 + int(x * 600)          # 200–800 Hz from logistic map
    audio_spin = int((s0 + s1 + s2) * 300)   # spinor energy → harmonic lift
    audio_quant = int(sum(q_bits) * 150)     # quantum states → resonance bumps
    audio_buttons = int(button_pressure * 250)
    audio_temp = int(temp_influence * 200)   # Day 8: temperature → thermal resonance
    audio_tilt = int(tilt_influence * 300)   # Day 9: tilt/shake → mechanical resonance
    audio_beam = int(beam_influence * 350)   # Day 10: beam break → detection spike
    audio_dip = int(dip_switch_value * 400)  # Day 16: DIP switch → configuration tone

    freq = audio_base + audio_spin + audio_quant + audio_buttons + audio_temp + audio_tilt + audio_beam + audio_dip

    # Keep piezo in a sane range so the RP2040 doesn't brown-note itself
    if freq < 100:
        freq = 100
    elif freq > 4000:     # 4 kHz max: still audible, not ultrasonic hell
        freq = 4000
    if pir_state:
        freq += 200   # sudden motion = excite the system

    piezo.freq(freq)

    # Duty cycle:
    #  - 0 if final chaos vector is all-zero
    #  - else we give it a jittery amplitude to keep it alive-sounding
    if sum(final) == 0:
        piezo.duty_u16(0)
    else:
        # 1–5% duty range is loud enough but safe
        # jitter keeps the tone from sounding like a fixed beep
        duty = 1000 + int(2000 * random.random())
        piezo.duty_u16(duty)

    # -------------------------
    # Beam break override (alarm) — takes precedence when beam is broken
    # -------------------------
    # If beam is currently broken, override LEDs and piezo
    if beam.value() == 0:  # Beam is broken right now
        # All LEDs on (alarm state)
        red.value(1)
        amber.value(1)
        green.value(1)
        # Piezo scream (high frequency alarm)
        piezo.freq(3500)
        piezo.duty_u16(5000)

    # -------------------------
    # Immediate PIR override (burst) — takes precedence for the burst duration
    # -------------------------
    # If a PIR rising edge occurred recently, make a short immediate flinch:
    if time.ticks_diff(pir_burst_until, now) > 0:
        # override LEDs to full-on flinch (you can change behavior here)
        red.value(1)
        amber.value(1)
        green.value(1)
        # make piezo screech briefly (bounded)
        piezo.freq(1800)
        piezo.duty_u16(3000)
    # else keep the normal outputs (already set above)

    # -------------------------
    # Day 11: Update OLED display
    # -------------------------

    # Calculate delay and clock speed first (for display)
    max_delay = 0.5 - (pot_norm * 0.4999)   # 0.5s down to 0.0001s
    jitter_variation = max_delay * 0.02 * (random.random() - 0.5)
    delay = max(0.0001, max_delay + jitter_variation)  # Keep minimum at 0.1ms
    clock_speed = int(1.0 / delay)  # Hz
    print("DELAY:", delay, "s  CLOCK:", clock_speed, "Hz")

    # Calculate bit flip percentage (0-100)
    bit_flip_pct = int(flip_chance * 100)

    oled.fill(0)  # Clear display

    # Line 1: Compact temp and chaos (y=0) + Clock speed on right
    oled.text("T:{:.0f} X:{:.2f}".format(current_temp, x), 0, 0)
    oled.text("C:{}".format(clock_speed), 88, 0)  # Right side

    # Line 2: Sensors + freq (y=8) + DIP count on right
    m = "M" if pir_state else "."
    t = "T" if tilt_state == 0 else "."
    b = "B" if beam_state == 0 else "."
    oled.text("{}{}{} {}Hz".format(m, t, b, freq), 0, 8)
    oled.text("D:{}".format(sum(dip_switches)), 96, 8)  # DIP switch count (0-5)

    # Line 3: LED states (y=16)
    r = "R" if red.value() else "."
    a = "A" if amber.value() else "."
    g = "G" if green.value() else "."
    oled.text("LED:{}{}{}".format(r, a, g), 0, 16)

    # Update wave buffer with chaos value (normalized to 0-7 for 8-pixel height)
    wave_value = int(x * 7)  # 0-7 pixels
    wave_buffer[wave_index] = wave_value
    wave_index = (wave_index + 1) % 128

    # Draw scrolling waveform as connected line (bottom 8 pixels, y=24-31)
    for i in range(127):  # Stop at 127 to avoid index out of range
        # Calculate buffer indices for scrolling effect
        buf_idx1 = (wave_index + i) % 128
        buf_idx2 = (wave_index + i + 1) % 128

        # Get two consecutive wave values
        y1 = 31 - wave_buffer[buf_idx1]  # Draw from bottom up
        y2 = 31 - wave_buffer[buf_idx2]

        # Draw line between consecutive points
        oled.line(i, y1, i + 1, y2, 1)

    oled.show()  # Update display

    # -------------------------
    # Day 12: Update WS2812 LED Strip (Waveform Bar Graph)
    # -------------------------
    # Display waveform as vertical bar graph
    for i in range(NUM_LEDS):
        # Get waveform value for this LED position
        # Sample from wave_buffer (spread across 128 buffer)
        buf_idx = (wave_index + i * (128 // NUM_LEDS)) % 128
        wave_height = wave_buffer[buf_idx]  # 0-7

        # Boost and scale: amplify waveform to use full 15 LED height
        # Add base offset and amplify the wave value
        boosted_height = wave_height + 3  # Add 3 to minimum (raises floor)
        scaled_height = int((boosted_height / 10.0) * NUM_LEDS)  # Scale to full strip
        scaled_height = min(NUM_LEDS, scaled_height)  # Cap at max LEDs

        if i < scaled_height:
            # LED is ON - gradient from blue (bottom) to purple/red (top)
            r = int(i * (200 / NUM_LEDS))      # More red toward top
            g = 0
            b = int(200 - i * (150 / NUM_LEDS))  # More blue toward bottom
            strip[i] = (r, g, b)
        else:
            # LED is OFF
            strip[i] = (0, 0, 0)

    strip.write()  # Update the strip

    # -------------------------
    # Day 17: Update 2x Addressable RGB LEDs
    # -------------------------
    # LED 0: Chaos rainbow (cycles through hue)
    # LED 1: Sensor activity (warm=active, cool=calm)
    update_rgb_leds(x, dip_switch_value, tilt_energy, beam_break_energy)

    # -------------------------
    # Day 18: Update 12-LED RGB Ring
    # -------------------------
    # Rotating rainbow pattern on slave Pico
    update_ring(x, spin, total_button_pressure)

    # -------------------------
    # Jitter — FASTER timing, aggressive pot control
    # -------------------------
    # Delay was already calculated before display update
    time.sleep(delay)
