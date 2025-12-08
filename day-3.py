# fused_chaos_with_quantum_and_dirac_influence.py (Day 3)
# MicroPython on RP2040 (Pico)
# Day 2 + Day 3 buttons

from machine import Pin, ADC
import time
import random
import math

# -------------------------
# Dirac toy spinor (4 components) - now button-influenced
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
# Hardware / LED setup
# -------------------------
red = Pin(18, Pin.OUT)
amber = Pin(19, Pin.OUT)
green = Pin(20, Pin.OUT)
leds = [red, amber, green]

# Floating ADC for real environmental entropy (leave pin unconnected for noise)
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
# Chaos & mutation seeds
# -------------------------
x = random.random()  # logistic map seed
r = 3.9              # initial chaotic parameter (will be modulated)
mut_state = [0, 1, 0]  # internal evolving bitfield

# -------------------------
# Quantum-like (Rabi) parameters
# -------------------------
OMEGA_BASE = 2.0 * math.pi * 2.0  # base angular frequency (~2 Hz)
OMEGA = OMEGA_BASE
quantum_phase = 0.0

# time tracking
last_t = time.ticks_ms()

# -------------------------
# Helper: manual shuffle (MicroPython has no random.shuffle)
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
    # -- elapsed time --
    now = time.ticks_ms()
    dt_ms = time.ticks_diff(now, last_t)
    if dt_ms < 0:
        dt_ms = 0
    dt = dt_ms / 1000.0
    last_t = now
    t = now / 1000.0  # seconds (for spinor)

    # -------------------------
    # Read buttons
    # -------------------------
    button_bits = read_buttons()
    button_pressure = sum(button_bits) / 3.0

    # -- Dirac spinor influence (normalize to 0..1) - now button-modulated --
    spin = fake_dirac_spinor(t, button_bits)
    s0 = (spin[0] + 1.0) / 2.0  # influences r
    s1 = (spin[1] + 1.0) / 2.0  # influences OMEGA
    s2 = (spin[2] + 1.0) / 2.0  # influences mutation rate
    s3 = (spin[3] + 1.0) / 2.0  # influences timing jitter

    # Create 3 influence bits from spinor (stochastic "measurement")
    spin_bits = [1 if s0 > random.random() else 0,
                 1 if s1 > random.random() else 0,
                 1 if s2 > random.random() else 0]

    # -- Environmental noise (ADC) – extract a few noisy bits --
    n = noise.read_u16()
    noise_bits = [
        (n >> 0) & 1,
        (n >> 1) & 1,
        (n >> 2) & 1
    ]

    # -- Modulate logistic parameter r with spinor channel s0 and button pressure
    r = 3.5 + s0 * 0.49 + button_pressure * 0.1

    # -- Logistic map chaos --
    x = r * x * (1 - x)  # ensure x stays in (0,1)
    if not (0.0 < x < 1.0):
        x = random.random()
    chaos_index = int(x * 3)  # 0..2
    chaos_bits = [1, 1, 1]
    chaos_bits[chaos_index] = 0  # treat 0 as "active" if you prefer active-low

    # -- Mutation / drift, influenced by spinor s2 and buttons
    mutate_chance = 0.05 + s2 * 0.35 + button_pressure * 0.2
    if random.random() < mutate_chance:
        idx = random.randint(0, 2)
        mut_state[idx] = random.randint(0, 1)

    # occasional manual shuffle also biased by s2 and buttons
    if random.random() < (0.15 + s2 * 0.2 + button_pressure * 0.2):
        manual_shuffle(mut_state)

    # -- Rabi-like oscillator, OMEGA modulated by spinor s1 and buttons
    OMEGA = OMEGA_BASE * (1.0 + (s1 - 0.5) * 0.6 + button_pressure * 0.3)
    if OMEGA < 0.1:
        OMEGA = 0.1

    quantum_phase += OMEGA * dt
    if quantum_phase > 1e6:
        quantum_phase = quantum_phase % (2.0 * math.pi)

    P_base = math.sin(0.5 * quantum_phase) ** 2
    q_bits = []
    for i in range(3):
        offset = i * 0.7
        p = math.sin(0.5 * (quantum_phase + offset)) ** 2
        # collapse probability, but bias with spin bit and button
        bias = 0.05 * spin_bits[i] + 0.1 * button_bits[i]
        q_bits.append(1 if (p + bias > random.random()) else 0)

    # -- Fusion: combine all sources (noise, chaos, mut_state, q_bits, spin_bits, button_bits) --
    final = []
    for i in range(3):
        mixed = (
            noise_bits[i]      # Environmental randomness
            ^ chaos_bits[i]    # Deterministic chaos
            ^ mut_state[i]     # Mutation memory
            ^ q_bits[i]        # Quantum oscillator
            ^ spin_bits[i]     # Dirac spinor
            ^ button_bits[i]   # Day 3: User input
        )
        # extra spice: spinor s3 and buttons add flip chance
        if random.random() < (0.02 + 0.06 * s3 + 0.05 * button_pressure):
            mixed ^= 1
        final.append(mixed)

    # -- Output to LEDs (assume HIGH=1 > LED on; invert if your wiring is active-low) --
    red.value(final[0])
    amber.value(final[1])
    green.value(final[2])

    # -- Unstable timing: combine P_base, Dirac s3, and buttons into jitter
    jitter = 0.003 + P_base * 0.05 + s3 * 0.03 + button_pressure * 0.04
    rand_sleep = random.uniform(jitter, jitter + 0.12)
    time.sleep(rand_sleep)
