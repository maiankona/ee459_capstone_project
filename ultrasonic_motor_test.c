#include <avr/io.h>
#include <util/delay.h>

// Pin Definitions
#define TRIG_PIN PB7
#define ECHO_PIN PD2
#define MOTOR_A_PIN1 PD6
#define MOTOR_A_PIN2 PD7
#define MOTOR_B_PIN1 PD4
#define MOTOR_B_PIN2 PD5
#define MOTOR_A_PWM PB2  // OC1B
#define MOTOR_B_PWM PD3  // OC2B

// Constants
#define STOP_DISTANCE 100  // cm
#define FORWARD_SPEED 255  // Full speed
#define PULSE_TIMEOUT 60000  // Maximum time to wait for echo (in timer counts)

void init_ultrasonic(void) {
    // Set trigger pin as output
    DDRB |= (1 << TRIG_PIN);
    // Set echo pin as input
    DDRD &= ~(1 << ECHO_PIN);
}

void init_motors(void) {
    // Set Motor A and B direction pins as outputs
    DDRD |= (1 << MOTOR_A_PIN1)
          | (1 << MOTOR_A_PIN2)
          | (1 << MOTOR_B_PIN1)
          | (1 << MOTOR_B_PIN2);
}

void init_pwm(void) {
    // --- Motor A on OC1B (PB2) ---
    DDRB |= (1 << MOTOR_A_PWM);
    TCCR1A = (1 << COM1B1) | (1 << WGM10);    // Fast PWM, 8-bit, non-inverting
    TCCR1B = (1 << WGM12) | (1 << CS11);      // prescaler = 8
    OCR1B = 0;

    // --- Motor B on OC2B (PD3) ---
    DDRD |= (1 << MOTOR_B_PWM);
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, 8-bit, non-inverting
    TCCR2B = (1 << CS21);                         // prescaler = 8
    OCR2B = 0;
}

float get_distance(void) {
    // Send trigger pulse
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);
    
    // Wait for echo start
    while (!(PIND & (1 << ECHO_PIN)));
    
    // Measure echo duration
    uint16_t pulse_start = TCNT1;
    while (PIND & (1 << ECHO_PIN)) {
        if (TCNT1 - pulse_start > PULSE_TIMEOUT) return 999.0; // Timeout
    }
    uint16_t pulse_end = TCNT1;
    
    // Calculate distance in cm (speed of sound = 343m/s)
    float distance = (pulse_end - pulse_start) * 0.0343 / 2;
    return (distance > 999.0) ? 999.0 : distance;
}

void set_motor_a_forward(void) {
    PORTD &= ~(1 << MOTOR_A_PIN1);
    PORTD |=  (1 << MOTOR_A_PIN2);
}

void set_motor_b_forward(void) {
    PORTD &= ~(1 << MOTOR_B_PIN1);
    PORTD |=  (1 << MOTOR_B_PIN2);
}

void set_motor_a_speed(uint8_t speed) {
    OCR1B = speed;
}

void set_motor_b_speed(uint8_t speed) {
    OCR2B = speed;
}

void set_motors_forward(uint8_t speed) {
    set_motor_a_forward();
    set_motor_b_forward();
    set_motor_a_speed(speed);
    set_motor_b_speed(speed);
}

void stop_motors(void) {
    // Stop both motors
    PORTD &= ~((1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2) |
               (1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2));
    OCR1B = 0;  // Motor A
    OCR2B = 0;  // Motor B
}

int main(void) {
    // Initialize hardware
    init_ultrasonic();
    init_motors();
    init_pwm();
    
    // Initial delay to let hardware settle
    _delay_ms(1000);
    
    while (1) {
        // Start moving forward at full speed
        set_motors_forward(FORWARD_SPEED);
        
        // Keep moving until obstacle detected
        while (1) {
            float distance = get_distance();
            
            // If obstacle detected within stop distance, stop motors
            if (distance <= STOP_DISTANCE) {
                stop_motors();
                break;
            }
            
            // Small delay between distance measurements
            _delay_ms(100);
        }
        
        // Wait for obstacle to be removed
        while (get_distance() <= STOP_DISTANCE) {
            _delay_ms(100);
        }
    }
    
    return 0;
} 