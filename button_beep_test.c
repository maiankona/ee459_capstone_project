/*
 * Button Beep Test for ATmega328P
 * - Button 1 (PC1) -> 1000Hz beep (PCINT9)
 * - Button 2 (PC2) -> 1500Hz beep (PCINT10)
 * - Button 3 (PC3) -> 2000Hz beep (PCINT11)
 * - Speaker on PB1 (OC1A)
 * - ATmega328P @ 7.3728 MHz
 */

#define F_CPU 7372800UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Button pins
#define BUTTON1_PIN PC1
#define BUTTON2_PIN PC2
#define BUTTON3_PIN PC3

// Speaker pin
#define SPEAKER_PIN PB1

// Beep parameters
#define BEEP_DURATION_MS 1000  // Longer beep duration
#define PRESCALER 8UL

// Button debouncing
volatile uint8_t button1_pressed = 0;
volatile uint8_t button2_pressed = 0;
volatile uint8_t button3_pressed = 0;
volatile uint8_t debounce_timer = 0;
#define DEBOUNCE_TIME 5  // ms

// Function to play a beep at specified frequency
void play_beep(uint16_t frequency) {
    // Calculate OCR1A value for the desired frequency
    uint16_t ocr1a_val = ((F_CPU / (2UL * PRESCALER * frequency)) - 1);
    
    // Configure Timer1 for sound generation
    TCCR1A = (1 << COM1A0);  // Toggle OC1A on compare match
    TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC mode, prescaler = 8
    
    // Set the compare value
    OCR1A = ocr1a_val;
    
    // Wait for duration
    _delay_ms(BEEP_DURATION_MS);
    
    // Stop sound
    TCCR1A = 0;
    TCCR1B = 0;
    PORTB &= ~(1 << SPEAKER_PIN);
}

// Initialize buttons with interrupts
void init_buttons(void) {
    // Set button pins as inputs with pull-up
    DDRC &= ~((1 << BUTTON1_PIN) | (1 << BUTTON2_PIN) | (1 << BUTTON3_PIN));
    PORTC |= (1 << BUTTON1_PIN) | (1 << BUTTON2_PIN) | (1 << BUTTON3_PIN);
    
    // Enable pin change interrupts for all buttons
    PCICR = (1 << PCIE1);  // Enable PCINT[8:14]
    PCMSK1 = (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11); // Enable PCINT9,10,11 (PC1,2,3)
}

// Initialize speaker
void init_speaker(void) {
    // Set speaker pin as output
    DDRB |= (1 << SPEAKER_PIN);
    PORTB &= ~(1 << SPEAKER_PIN);  // Ensure speaker is initially off
    
    // Test beep at startup
    play_beep(1000);
    _delay_ms(1000);
    play_beep(1500);
    _delay_ms(1000);
    play_beep(2000);
}

// Initialize Timer0 for debouncing
void init_timer0(void) {
    // Configure Timer0 for 1ms interrupts
    TCCR0A = (1 << WGM01);  // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00);  // prescaler = 64
    OCR0A = (F_CPU / 64 / 1000) - 1;  // 1ms period
    TIMSK0 = (1 << OCIE0A);  // Enable Compare Match A interrupt
}

// Pin Change Interrupt for all buttons
ISR(PCINT1_vect) {
    // Check which button was pressed
    if (!(PINC & (1 << BUTTON1_PIN))) {
        button1_pressed = 1;
    }
    if (!(PINC & (1 << BUTTON2_PIN))) {
        button2_pressed = 1;
    }
    if (!(PINC & (1 << BUTTON3_PIN))) {
        button3_pressed = 1;
    }
}

// Timer0 Compare Match A interrupt for debouncing
ISR(TIMER0_COMPA_vect) {
    if (debounce_timer > 0) {
        debounce_timer--;
    } else {
        if (button1_pressed) {
            play_beep(1000);
            button1_pressed = 0;
            debounce_timer = DEBOUNCE_TIME;
        }
        if (button2_pressed) {
            play_beep(1500);
            button2_pressed = 0;
            debounce_timer = DEBOUNCE_TIME;
        }
        if (button3_pressed) {
            play_beep(2000);
            button3_pressed = 0;
            debounce_timer = DEBOUNCE_TIME;
        }
    }
}

int main(void) {
    // Initialize hardware
    init_buttons();
    init_speaker();
    init_timer0();
    
    // Enable global interrupts
    sei();
    
    // Main loop
    while (1) {
        // Just wait for button interrupts
    }
    
    return 0;
} 