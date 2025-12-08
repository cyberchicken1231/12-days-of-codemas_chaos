# fused_chaos_with_quantum_and_dirac_influence.py (Day 5)
# MicroPython on RP2040 (Pico)
# Day 4 + Day 5 piezo

from machine import Pin, ADC, PWM
import time
import random
import math

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
    pot_norm = pot_raw / 65535.0

    pot_flip = pot_norm * 0.25

    # -------------------------
    # Dirac spinor influence
    # -------------------------
    spin = fake_dirac_spinor(t, button_bits)
    s0 = (spin[0] + 1) / 2
    s1 = (spin[1] + 1) / 2
    s2 = (spin[2] + 1) / 2
    s3 = (spin[3] + 1) / 2

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
    r = 3.5 + s0 * 0.49 + button_pressure * 0.1
    x = r * x * (1 - x)
    if not (0 < x < 1):
        x = random.random()

    chaos_index = int(x * 3)
    chaos_bits = [1, 1, 1]
    chaos_bits[chaos_index] = 0

    # -------------------------
    # Mutation
    # -------------------------
    mutate_chance = 0.05 + s2 * 0.35 + button_pressure * 0.2
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
    if quantum_phase > 1e6:
        quantum_phase %= 2 * math.pi

    q_bits = []
    for i in range(3):
        off = i * 0.7
        p = math.sin(0.5 * (quantum_phase + off)) ** 2
        bias = 0.05 * spin_bits[i] + 0.1 * button_bits[i]
        q_bits.append(1 if (p + bias > random.random()) else 0)

    # -------------------------
    # Fusion - XOR all sources together
    # -------------------------
    final = []
    for i in range(3):
        mixed = (
            noise_bits[i]
            ^ chaos_bits[i]
            ^ mut_state[i]
            ^ q_bits[i]
            ^ spin_bits[i]
            ^ button_bits[i]
        )

        flip_chance = 0.02 + 0.06 * s3 + 0.05 * button_pressure + pot_flip
        if random.random() < flip_chance:
            mixed ^= 1

        final.append(mixed)

    # -------------------------
    # Output LEDs
    # -------------------------
    red.value(final[0])
    amber.value(final[1])
    green.value(final[2])

    # -------------------------
    # Day 5: Chaos audio output
    # -------------------------
    # Pick frequency based on fusion of chaos components
    audio_base = 200 + int(x * 600)          # 200–800 Hz from logistic map
    audio_spin = int((s0 + s1 + s2) * 300)   # spinor energy → harmonic lift
    audio_quant = int(sum(q_bits) * 150)     # quantum states → resonance bumps
    audio_buttons = int(button_pressure * 250)

    freq = audio_base + audio_spin + audio_quant + audio_buttons

    # Keep piezo in a sane range
    if freq < 100:
        freq = 100
    elif freq > 4000:
        freq = 4000

    piezo.freq(freq)

    # Duty cycle: 0 if final chaos vector is all-zero
    if sum(final) == 0:
        piezo.duty_u16(0)
    else:
        # 1–5% duty range is loud enough but safe
        duty = 1000 + int(2000 * random.random())
        piezo.duty_u16(duty)

    # -------------------------
    # Jitter
    # -------------------------
    jitter = 0.003 + (math.sin(0.5 * quantum_phase)**2) * 0.05
    jitter += s3 * 0.03
    jitter += button_pressure * 0.04

    base_delay = random.uniform(jitter, jitter + 0.12)

    # Pot controls max loop delay
    max_delay = 0.001 + pot_norm * 0.149
    delay = min(base_delay, max_delay)
    time.sleep(delay)
