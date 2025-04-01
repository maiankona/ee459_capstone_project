""" 
EE 250L Lab 02: SparkFun Ultrasonic Finder Implementation

Team members:
Maia Nkonabang
"""

import sys
import time
import RPi.GPIO as GPIO

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Constants for Ultrasonic Finder
TRIG_PIN = 23  # GPIO pin for trigger (adjust as needed)
ECHO_PIN = 24  # GPIO pin for echo (adjust as needed)

# Setup GPIO pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    """Measure distance using SparkFun Ultrasonic Finder"""
    # Ensure trigger is low first
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.1)  # Let sensor settle
    
    # Send 10µs pulse
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    
    # Measure echo duration
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    
    # Calculate distance (cm)
    distance = pulse_duration * 17150  # 34300 cm/s divided by 2 for round trip
    distance = round(distance, 2)
    
    return distance

if __name__ == '__main__':
    try:
        while True:
            # Get distance measurement
            distance = get_distance()
            
            # Print distance to console
            print(f"Distance: {distance} cm")
            
            # Add brief delay between measurements
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print("\nProgram stopped")
    finally:
        GPIO.cleanup()