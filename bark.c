/*
 * timed_beep.c
 * —————————
 * ATmega328P @ 7.3728 MHz
 * Beep once per interval on PB1 (OC1A).
 * Interval is 10 000 ms for now; uncomment the 24 h line for real.
 */

#define F_CPU       7372800UL
#include <avr/io.h>
#include <util/delay.h>

// tone parameters
#define BEEP_FREQ   1000UL    // 1 kHz beep
#define PRESCALER     8UL     // Timer1 prescaler
// OCR1A = (F_CPU / (2 * PRESCALER * BEEP_FREQ)) - 1
#define OCR1A_VAL  ((F_CPU / (2UL * PRESCALER * BEEP_FREQ)) - 1)

// how long to beep (ms)
#define BEEP_DURATION_MS  500

int main(void) {
    // PB1/OC1A as output
    DDRB |= (1 << PB1);

    // preload compare
    OCR1A = OCR1A_VAL;

    for (;;) {
        // ────── interval delay ──────
        _delay_ms(10000);       // test: 10 s
        // _delay_ms(24UL * 60UL * 60UL * 1000UL);  // real: 24 h
    
        // ────── start tone ──────
        // CTC mode + toggle OC1A on compare + prescaler=8
        TCCR1A = (1 << COM1A0);
        TCCR1B = (1 << WGM12) | (1 << CS11);

        _delay_ms(BEEP_DURATION_MS);

        // ────── stop tone ──────
        TCCR1A = 0;
        TCCR1B = 0;
    }
    return 0;
}
