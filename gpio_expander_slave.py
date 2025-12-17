# GPIO Expander Slave - Raspberry Pi Pico #2
# MicroPython on RP2040 (Pico)
#
# This Pico acts as a UART slave device to expand GPIO for the main Pico.
# It reads a 5-way DIP switch and sends the state over UART when requested.
#
# Wiring between Picos (UART connection):
# - Main Pico GP12 (UART0 TX) --> Slave Pico GP1  (UART0 RX) [Physical pin 16 -> pin 2]
# - Main Pico GP13 (UART0 RX) --> Slave Pico GP0  (UART0 TX) [Physical pin 17 -> pin 1]
# - Main Pico GND --> Slave Pico GND [Any GND pin]
#
# Make sure both Picos share a common ground!

from machine import Pin, UART
import time

# -------------------------
# UART Configuration
# -------------------------
# UART0 on GPIO 0 (TX) and GPIO 1 (RX)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# -------------------------
# 5-way DIP Switch Configuration
# -------------------------
# Using GPIO 2-6 for the 5 DIP switches
# Each switch connects GPIO to GND when ON, floating when OFF
# Using internal pull-ups: HIGH (1) = OFF, LOW (0) = ON
dip_pins = [
    Pin(2, Pin.IN, Pin.PULL_UP),  # DIP switch 1 (GPIO 2, Physical pin 4)
    Pin(3, Pin.IN, Pin.PULL_UP),  # DIP switch 2 (GPIO 3, Physical pin 5)
    Pin(4, Pin.IN, Pin.PULL_UP),  # DIP switch 3 (GPIO 4, Physical pin 6)
    Pin(5, Pin.IN, Pin.PULL_UP),  # DIP switch 4 (GPIO 5, Physical pin 7)
    Pin(6, Pin.IN, Pin.PULL_UP),  # DIP switch 5 (GPIO 6, Physical pin 9)
]

# Status LED (built-in)
status_led = Pin(25, Pin.OUT)
status_led.value(1)  # Turn on to show slave is running

def read_dip_switches():
    """Read all 5 DIP switches and return as byte
    Returns byte with bits 0-4 representing switches 1-5
    Bit = 1 when switch is ON (closed to ground)
    Bit = 0 when switch is OFF (open/floating)
    """
    state = 0
    for i, pin in enumerate(dip_pins):
        # Invert because pull-up: pin LOW (0) = switch ON
        if pin.value() == 0:  # Switch ON (closed to GND)
            state |= (1 << i)  # Set bit
    return state

print("=" * 40)
print("GPIO Expander Slave - Pico #2")
print("=" * 40)
print("UART0 TX: GPIO 0 (Physical pin 1)")
print("UART0 RX: GPIO 1 (Physical pin 2)")
print("Baudrate: 9600")
print("DIP switches: GPIO 2-6 (Physical pins 4-9)")
print("=" * 40)
print("Waiting for requests from master...\n")

# -------------------------
# Protocol: Simple request-response
# -------------------------
# Master sends 'R' (Read request)
# Slave responds with single byte containing DIP switch state
#
# Optional: Master can send 'S' (Status) for diagnostics
# -------------------------

last_state = 0xFF  # Track last state for change detection
request_count = 0  # Count requests for diagnostics

while True:
    # Check if master sent a request
    if uart.any():
        request = uart.read(1)
        if request:
            cmd = request[0]

            if cmd == ord('R'):  # Read request
                # Read DIP switches
                state = read_dip_switches()

                # Send state back to master
                uart.write(bytes([state]))

                # Blink LED on request
                status_led.toggle()

                # Print for debugging (only when state changes)
                if state != last_state:
                    print("DIP: {:05b} (0x{:02X}) - Sent to master".format(state, state))
                    last_state = state

                request_count += 1

            elif cmd == ord('S'):  # Status request
                # Send diagnostic info
                state = read_dip_switches()
                status_msg = "OK:{},REQ:{}\n".format(state, request_count)
                uart.write(status_msg.encode())
                print("Status request received - sent:", status_msg.strip())

    # Small delay to avoid busy-waiting
    time.sleep_ms(1)
