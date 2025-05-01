#include <avr/io.h>
#include <util/delay.h>

// Pin Definitions
#define TRIG_PIN PD2  // Ultrasonic trigger
#define ECHO_PIN PD3  // Ultrasonic echo
#define SPEAKER_PIN PB0  // Speaker output

// Constants
#define STOP_DISTANCE 30  // cm
#define PULSE_TIMEOUT 60000  // timer counts
#define BEEP_FREQ 2000      // Hz
#define BEEP_DURATION 200   // ms

// Function to initialize ultrasonic sensor
void init_ultrasonic(void) {
    DDRD |= (1 << TRIG_PIN);    // Set trigger as output
    DDRD &= ~(1 << ECHO_PIN);   // Set echo as input
}

// Function to initialize speaker
void init_speaker(void) {
    DDRB |= (1 << SPEAKER_PIN);  // Set speaker pin as output
}

// Function to play a beep
void play_beep(void) {
    // Configure Timer0 for sound generation
    TCCR0A = (1 << COM0A0);  // Toggle OC0A on compare match
    TCCR0B = (1 << WGM01);   // CTC mode
    OCR0A = (F_CPU / (2 * BEEP_FREQ)) - 1;
    
    // Enable Timer0
    TCCR0B |= (1 << CS01);  // Prescaler 8
    
    // Wait for duration
    _delay_ms(BEEP_DURATION);
    
    // Stop sound
    TCCR0B &= ~(1 << CS01);
    PORTB &= ~(1 << SPEAKER_PIN);
}

// Function to get distance from ultrasonic sensor
uint16_t get_distance(void) {
    uint16_t pulse_width = 0;
    
    // Send trigger pulse
    PORTD |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);
    
    // Wait for echo pulse
    while (!(PIND & (1 << ECHO_PIN)));
    
    // Measure pulse width
    while (PIND & (1 << ECHO_PIN)) {
        pulse_width++;
        if (pulse_width > PULSE_TIMEOUT) {
            return 0;  // Timeout
        }
        _delay_us(1);
    }
    
    // Convert to cm (assuming speed of sound is 343 m/s)
    return pulse_width / 58;
}

int main(void) {
    // Initialize hardware
    init_ultrasonic();
    init_speaker();
    
    while (1) {
        // Get distance
        uint16_t distance = get_distance();
        
        // If object is detected within STOP_DISTANCE, play beep
        if (distance > 0 && distance < STOP_DISTANCE) {
            play_beep();
        }
        
        // Small delay to prevent too frequent updates
        _delay_ms(100);
    }
    
    return 0;
} 