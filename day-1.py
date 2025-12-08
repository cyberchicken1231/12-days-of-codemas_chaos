# Day 1: Basic Setup Test
# MicroPython on RP2040 (Pico H)
# Components: Pico H, breadboard, USB cable only
# Test the built-in LED on the Pico H

from machine import Pin
import time

# -------------------------
# Built-in LED on Pico (GPIO 25)
# -------------------------
led = Pin(25, Pin.OUT)

# -------------------------
# Main loop - Simple blink
# -------------------------
counter = 0
while True:
    led.value(1)
    print("Day 1: Blink ON - Count:", counter)
    time.sleep(0.5)

    led.value(0)
    print("Day 1: Blink OFF")
    time.sleep(0.5)

    counter += 1
