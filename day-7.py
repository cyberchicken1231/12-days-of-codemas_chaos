# fused_chaos_with_quantum_and_dirac_influence.py (Day 4 integrated)
# MicroPython on RP2040 (Pico)

from machine import Pin, ADC
import time
import random
import math
import onewire
import ds18x20

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
# Day 3: Buttons
# -------------------------
button_pins = [13, 8, 3]
buttons = [Pin(p, Pin.IN, Pin.PULL_UP) for p in button_pins]

def read_buttons():
    return [
        0 if buttons[0].value() else 1,
        0 if buttons[1].value() else 1,
        0 if buttons[2].value() else 1,
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
# Initialize 1-wire bus on GPIO 26
temp_pin = Pin(26)
temp_sensor = ds18x20.DS18X20(onewire.OneWire(temp_pin))

# Scan for DS18B20 devices
temp_roms = temp_sensor.scan()
print("Found DS18B20 devices:", temp_roms)

# Temperature tracking
last_temp_read = 0
current_temp = 20.0  # default starting temperature in Celsius
TEMP_READ_INTERVAL = 750  # read every 750ms (sensor conversion takes ~750ms)

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
    # Read potentiometer
    # -------------------------
    pot_raw = pot.read_u16()
    pot_norm = pot_raw / 65535.0      # 0.0 → 1.0

    # -------------------------
    # Day 6: Phototransistor noise
    # -------------------------
    photo_bits, photo_level = get_photo_noise()

    # -------------------------
    # Day 8: Temperature sensor read (non-blocking with interval)
    # -------------------------
    temp_influence = 0.0
    temp_bits = [0, 0, 0]

    if temp_roms and time.ticks_diff(now, last_temp_read) >= TEMP_READ_INTERVAL:
        try:
            # Start conversion
            temp_sensor.convert_temp()
            time.sleep_ms(750)  # Wait for conversion (DS18B20 needs ~750ms)

            # Read temperature
            current_temp = temp_sensor.read_temp(temp_roms[0])
            last_temp_read = now

            print("TEMP:", current_temp, "°C")

            # Normalize temperature to 0.0-1.0 range
            # Assume typical indoor range: 15°C to 35°C
            temp_norm = (current_temp - 15.0) / 20.0
            temp_norm = max(0.0, min(1.0, temp_norm))  # clamp to 0-1

            # Temperature influence on chaos (warmer = more chaotic)
            temp_influence = temp_norm * 0.3

            # Convert temperature to entropy bits
            # Use fractional part of temperature for randomness
            temp_frac = (current_temp * 100) % 8  # 0-7
            temp_bits = [
                (int(temp_frac) >> 0) & 1,
                (int(temp_frac) >> 1) & 1,
                (int(temp_frac) >> 2) & 1,
            ]
        except Exception as e:
            print("Temp read error:", e)

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

    # extra bit-flip energy
    pot_flip = pot_norm * 0.25

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
    # Adjust r parameter based on spinor, button input, and temperature (stays in chaotic regime)
    r = 3.5 + s0 * 0.49 + button_pressure * 0.1 + temp_influence * 0.15
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
    # Temperature adds thermal noise to mutation (warmer = more mutations)
    mutate_chance = 0.05 + s2 * 0.35 + button_pressure * 0.2 + temp_influence * 0.15
    if random.random() < mutate_chance:
        idx = random.randint(0, 2)
        mut_state[idx] = random.randint(0, 1)

    if random.random() < (0.15 + s2 * 0.2 + button_pressure * 0.2):
        manual_shuffle(mut_state)

    # -------------------------
    # Quantum oscillator
    # -------------------------
    OMEGA = OMEGA_BASE * (1.0 + (s1 - 0.5) * 0.6 + button_pressure * 0.3)
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
        )

        # Add probabilistic bit flip for extra unpredictability
        # Temperature adds thermal noise to flip chance
        flip_chance = 0.02 + 0.06 * s3 + 0.05 * button_pressure + pot_flip + pir_jitter + temp_influence * 0.1
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
    # Day 5: Chaos audio output
    # -------------------------
    # Pick a frequency based on the fusion of chaos components.
    # Piezo reacts to: logistic map + spinor + q-state + buttons + temperature
    audio_base = 200 + int(x * 600)          # 200–800 Hz from logistic map
    audio_spin = int((s0 + s1 + s2) * 300)   # spinor energy → harmonic lift
    audio_quant = int(sum(q_bits) * 150)     # quantum states → resonance bumps
    audio_buttons = int(button_pressure * 250)
    audio_temp = int(temp_influence * 200)   # Day 8: temperature → thermal resonance

    freq = audio_base + audio_spin + audio_quant + audio_buttons + audio_temp

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
    # Jitter — variable timing influenced by quantum phase, spinor, and buttons
    # -------------------------
    # Base timing jitter from quantum oscillator
    jitter = 0.003 + (math.sin(0.5 * quantum_phase)**2) * 0.05
    jitter += s3 * 0.03            # Spinor influence
    jitter += button_pressure * 0.04  # Button influence

    # Random delay within jitter range
    base_delay = random.uniform(jitter, jitter + 0.12)

    # Apply speed multiplier from potentiometer (time dilation effect)
    # LOW-LATENCY PIR LOOP FIX: cap the delay so PIR pulses are not missed
    # Pot controls max loop delay from 1ms → 150ms
    max_delay = 0.001 + pot_norm * 0.149   # 0.001–0.150 seconds
    # Jitter still applies but capped by max_delay
    delay = min(base_delay, max_delay)
    time.sleep(delay)
