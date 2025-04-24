/* Real Bark Playback on ATmega328P
 *  - ATmega328P @ 7.3728 MHz
 *  - Speaker on PB1 (OC1A), via resistor or transistor driver
 *  - Sample rate: 8 kHz, 4 bits/sample packed → 2 000 bytes per 0.5 s bark
 */

#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// Include the generated bark data
#include "bark_packed.h"

#define SAMPLE_RATE 8000
#define BUFFER_SIZE 64

volatile uint8_t audio_buffer[BUFFER_SIZE];
volatile uint8_t buffer_index = 0;
volatile uint8_t is_playing = 0;

// -----------------------------------------------------------------------------
// 2) Playback parameters
#define SPEAKER_PIN   PB1
#define PRESCALER0    64
#define OCR0A_VAL     ((F_CPU/(PRESCALER0*SAMPLE_RATE))-1)  // ≈13

// Playback state
volatile uint32_t sample_index = 0;  // counts nibbles
volatile uint8_t  nibble_flip  = 0;  // 0→high nibble, 1→low nibble

// -----------------------------------------------------------------------------
// 3) PWM on Timer1 (OC1A / PB1) – 8‑bit fast PWM, prescaler=8
void init_audio(void) {
    // Configure Timer1 for PWM output
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << CS10);
    DDRB |= (1 << PB1); // Set PB1 as output for PWM
}

// -----------------------------------------------------------------------------
// 4) Timer0 in CTC mode to generate SAMPLE_RATE interrupts
void init_timer0(void) {
    OCR0A  = OCR0A_VAL;
    TCCR0A = (1 << WGM01);                       // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00);          // prescaler = 64
    TIMSK0 = (1 << OCIE0A);                      // enable compare match ISR
}

// -----------------------------------------------------------------------------
// 5) ISR: fired at SAMPLE_RATE; extract next 4‑bit sample and output via PWM
ISR(TIMER0_COMPA_vect) {
    if (!is_playing) {
        OCR1A = 128;  // Middle value for silence
        return;
    }
    
    uint16_t byte_idx = sample_index >> 1;          // each byte = 2 samples
    if (byte_idx >= sizeof(bark_data)) {
        // End of playback
        is_playing = 0;
        OCR1A = 128;  // Middle value for silence
        return;
    }

    // Read the packed byte from flash
    uint8_t packed = pgm_read_byte(&bark_data[byte_idx]);

    // Extract high or low nibble
    uint8_t sample4 = nibble_flip ? (packed & 0x0F) : (packed >> 4);

    // Toggle nibble state; advance index after low nibble
    nibble_flip ^= 1;
    if (nibble_flip == 0) {
        sample_index++;
    }

    // Scale 4‑bit (0–15) up to 8‑bit (0–240) and output
    // Use a lookup table for better quality
    static const uint8_t scale_table[] = {
        0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240
    };
    OCR1A = scale_table[sample4];
}

void play_bark(void) {
    is_playing = 1;
    for (uint16_t i = 0; i < sizeof(bark_data); i++) {
        OCR1A = pgm_read_byte(&bark_data[i]);
        _delay_ms(1);
    }
    is_playing = 0;
}

// -----------------------------------------------------------------------------
// 6) Main: init, enable interrupts, and loop
int main(void) {
    // Initial delay to let everything settle
    _delay_ms(1000);
    
    init_audio();
    init_timer0();
    sei();  // global interrupts on

    while (1) {
        play_bark();
        _delay_ms(1000); // Wait before playing again
    }
    return 0;
}