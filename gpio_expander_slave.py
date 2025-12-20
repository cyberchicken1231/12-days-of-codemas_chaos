# GPIO Expander Slave - Raspberry Pi Pico #2
# MicroPython on RP2040 (Pico)
#
# This Pico acts as a UART slave device to expand GPIO for the main Pico.
# Features:
# - Reads 5-way DIP switch (GP2-6)
# - Controls RGB LED 1 (GP7) - Day 17
# - Sends DIP switch state over UART when requested
# - Receives RGB color commands from main Pico
#
# Wiring between Picos (UART connection):
# - Main Pico GP12 (UART0 TX) --> Slave Pico GP1  (UART0 RX) [Physical pin 16 -> pin 2]
# - Main Pico GP13 (UART0 RX) --> Slave Pico GP0  (UART0 TX) [Physical pin 17 -> pin 1]
# - Main Pico GND --> Slave Pico GND [Any GND pin]
#
# Make sure both Picos share a common ground!

from machine import Pin, UART
import time
import neopixel

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

# -------------------------
# Day 17: RGB LED 1 (Activity Indicator)
# -------------------------
# WS2812 RGB LED on GPIO 7 (Physical pin 10)
rgb_led = neopixel.NeoPixel(Pin(7), 1)  # GPIO 7 - 1 LED
rgb_led[0] = (0, 0, 50)  # Dim blue on startup
rgb_led.write()

# -------------------------
# Day 18: 12-LED RGB Ring (Let It Glow)
# -------------------------
# 12x WS2812 RGB LEDs in ring formation on GPIO 8 (Physical pin 12)
ring = neopixel.NeoPixel(Pin(8), 12)  # GPIO 8 (pin 12) - 12 LEDs in ring
ring_offset = 0  # Rotation offset for spinning effect

def update_ring_pattern(hue_offset, brightness):
    """Update 12-LED ring with rotating rainbow
    hue_offset: 0-360 (rotation position)
    brightness: 0-255 (overall brightness)
    """
    global ring_offset
    ring_offset = hue_offset % 360

    for i in range(12):
        # Calculate hue for this LED (30 degrees per LED)
        hue = (ring_offset + i * 30) % 360

        # Convert to RGB using simple HSV approximation
        r, g, b = simple_hsv_to_rgb(hue, brightness)
        ring[i] = (r, g, b)

    ring.write()

def simple_hsv_to_rgb(hue, brightness):
    """Simple HSV to RGB conversion for ring
    hue: 0-360
    brightness: 0-255
    """
    h = hue % 360
    v = brightness / 255.0

    if h < 60:
        r, g, b = 1.0, h/60.0, 0
    elif h < 120:
        r, g, b = (120-h)/60.0, 1.0, 0
    elif h < 180:
        r, g, b = 0, 1.0, (h-120)/60.0
    elif h < 240:
        r, g, b = 0, (240-h)/60.0, 1.0
    elif h < 300:
        r, g, b = (h-240)/60.0, 0, 1.0
    else:
        r, g, b = 1.0, 0, (360-h)/60.0

    return (int(r*v*255), int(g*v*255), int(b*v*255))

# Initialize ring with startup pattern
for i in range(12):
    ring[i] = (0, 10, 20)  # Dim cyan
ring.write()

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
print("RGB LED: GPIO 7 (Physical pin 10)")
print("LED Ring: GPIO 8 (Physical pin 12) - 12 LEDs")
print("=" * 40)
print("Waiting for requests from master...\n")

# -------------------------
# Protocol: Simple request-response
# -------------------------
# Master sends 'R' (Read DIP switches)
# - Slave responds with single byte containing DIP switch state
#
# Master sends 'C' followed by 3 bytes (R, G, B) (Set RGB LED color)
# - Slave sets RGB LED and sends 'K' confirmation
#
# Master sends 'N' followed by 2 bytes (hue_offset, brightness) (Update ring pattern)
# - Slave updates 12-LED ring and sends 'K' confirmation
#
# Master sends 'S' (Status request - diagnostics)
# - Slave sends status string
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

            elif cmd == ord('C'):  # Set RGB LED color
                # Wait for 3 bytes (R, G, B)
                timeout = time.ticks_ms()
                while uart.any() < 3:
                    if time.ticks_diff(time.ticks_ms(), timeout) > 50:  # 50ms timeout
                        break
                    time.sleep_ms(1)

                if uart.any() >= 3:
                    # Read RGB values
                    rgb_data = uart.read(3)
                    r, g, b = rgb_data[0], rgb_data[1], rgb_data[2]

                    # Set RGB LED
                    rgb_led[0] = (r, g, b)
                    rgb_led.write()

                    # Send confirmation
                    uart.write(b'K')  # Single byte 'K' for OK

            elif cmd == ord('N'):  # Update ring pattern (riNg)
                # Wait for 2 bytes (hue_offset, brightness)
                timeout = time.ticks_ms()
                while uart.any() < 2:
                    if time.ticks_diff(time.ticks_ms(), timeout) > 50:  # 50ms timeout
                        break
                    time.sleep_ms(1)

                if uart.any() >= 2:
                    # Read ring parameters
                    ring_data = uart.read(2)
                    hue_offset = int((ring_data[0] / 255.0) * 360)  # 0-255 -> 0-360
                    brightness = ring_data[1]  # 0-255

                    # Update ring pattern
                    update_ring_pattern(hue_offset, brightness)

                    # Send confirmation
                    uart.write(b'K')  # Single byte 'K' for OK

            elif cmd == ord('S'):  # Status request
                # Send diagnostic info
                state = read_dip_switches()
                status_msg = "OK:{},REQ:{}\n".format(state, request_count)
                uart.write(status_msg.encode())
                print("Status request received - sent:", status_msg.strip())

    # Small delay to avoid busy-waiting
    time.sleep_ms(1)
