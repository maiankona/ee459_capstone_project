#define F_CPU 7372800UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Pin definitions
#define TRIG_DDR  DDRC
#define TRIG_PORT PORTC
#define TRIG_PIN  PC5
#define ECHO_DDR  DDRC
#define ECHO_PORT PORTC
#define ECHO_PIN  PC4
#define ECHO_PIN_REG PINC

// Safety thresholds (in centimeters)
#define WARNING_DISTANCE 50
#define CRITICAL_DISTANCE 20
#define MAX_DISTANCE 400
#define MIN_DISTANCE 2

// Reading configuration
#define NUM_READINGS 5
#define READING_INTERVAL 20
#define SMOOTHING_WINDOW 5

// Variables for distance calculation
volatile uint16_t pulse_duration = 0;
volatile uint8_t echo_received = 0;
uint16_t last_valid_distance = MAX_DISTANCE;
uint8_t error_count = 0;
#define MAX_ERRORS 3

// Array for distance history
uint16_t distance_history[SMOOTHING_WINDOW];
uint8_t history_index = 0;

// Timer1 setup for pulse measurement
void setup_timer1() {
    // Normal mode, no prescaler
    TCCR1B |= (1 << CS10);
}

void send_trigger() {
    // Send 10µs pulse
    TRIG_PORT |= (1 << TRIG_PIN);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_PIN);
}

uint16_t get_single_reading() {
    // Ensure trigger is low first
    TRIG_PORT &= ~(1 << TRIG_PIN);
    _delay_us(2);
    
    // Clear timer
    TCNT1 = 0;
    
    // Send trigger pulse
    send_trigger();
    
    // Wait for echo start
    while (!(ECHO_PIN_REG & (1 << ECHO_PIN)));
    TCNT1 = 0;  // Start counting
    
    // Wait for echo end
    while ((ECHO_PIN_REG & (1 << ECHO_PIN)) && TCNT1 < 60000);
    
    // Calculate distance (cm)
    // pulse_duration * 0.034 / 2 = (pulse_duration * 17) / 1000
    uint16_t distance = (TCNT1 * 17) / 1000;
    
    // Validate reading
    if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
        return distance;
    }
    return 0;  // Invalid reading
}

uint16_t get_distance() {
    uint16_t readings[NUM_READINGS];
    uint8_t valid_readings = 0;
    
    // Take multiple readings
    for (uint8_t i = 0; i < NUM_READINGS; i++) {
        uint16_t reading = get_single_reading();
        if (reading != 0) {
            readings[valid_readings++] = reading;
        }
        _delay_ms(READING_INTERVAL);
    }
    
    if (valid_readings == 0) {
        error_count++;
        if (error_count >= MAX_ERRORS) {
            return last_valid_distance;
        }
        return 0;
    }
    
    // Sort readings to find median
    for (uint8_t i = 0; i < valid_readings - 1; i++) {
        for (uint8_t j = i + 1; j < valid_readings; j++) {
            if (readings[j] < readings[i]) {
                uint16_t temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }
    
    // Get median value
    uint16_t median_distance = readings[valid_readings / 2];
    
    // Update distance history
    distance_history[history_index] = median_distance;
    history_index = (history_index + 1) % SMOOTHING_WINDOW;
    
    // Calculate average of history
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SMOOTHING_WINDOW; i++) {
        sum += distance_history[i];
    }
    
    last_valid_distance = sum / SMOOTHING_WINDOW;
    error_count = 0;
    
    return last_valid_distance;
}

void check_obstacle() {
    uint16_t distance = get_distance();
    
    if (distance == 0) {
        // Error handling
        return;
    }
    
    if (distance <= CRITICAL_DISTANCE) {
        // Critical obstacle detected
    } else if (distance <= WARNING_DISTANCE) {
        // Warning obstacle detected
    } else {
        // Clear path
    }
}

int main(void) {
    // Set up pins
    TRIG_DDR |= (1 << TRIG_PIN);
    ECHO_DDR &= ~(1 << ECHO_PIN);
    
    // Initialize timer
    setup_timer1();
    
    // Initialize distance history
    for (uint8_t i = 0; i < SMOOTHING_WINDOW; i++) {
        distance_history[i] = MAX_DISTANCE;
    }
    
    while (1) {
        check_obstacle();
        _delay_ms(100);
    }
    
    return 0;
} 