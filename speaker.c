#define F_CPU 7372800UL  // Define the clock frequency (adjust if necessary)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// I2C and RTC definitions
#define PCF8523_ADDR 0x68  // I2C address of PCF8523
#define SDA_PIN PB4        // I2C SDA pin
#define SCL_PIN PB5        // I2C SCL pin
#define I2C_DDR DDRB
#define I2C_PORT PORTB

// Pin definitions
#define SPEAKER_DDR  DDRB
#define SPEAKER_PORT PORTB
#define SPEAKER_PIN  PB1  // Using OC1A (Timer1 output)

// Sound parameters
#define BARK_FREQUENCY 1000
#define BARK_DURATION  200
#define BARK_COUNT     3
#define BARK_INTERVAL  500

// I2C functions
void i2c_init() {
    // Set SDA and SCL as outputs
    I2C_DDR |= (1 << SDA_PIN) | (1 << SCL_PIN);
    // Set both lines high
    I2C_PORT |= (1 << SDA_PIN) | (1 << SCL_PIN);
}

void i2c_start() {
    I2C_DDR |= (1 << SDA_PIN);  // SDA as output
    _delay_us(5);
    I2C_DDR |= (1 << SCL_PIN);  // SCL as output
    _delay_us(5);
    I2C_PORT &= ~(1 << SDA_PIN);  // SDA low
    _delay_us(5);
    I2C_PORT &= ~(1 << SCL_PIN);  // SCL low
    _delay_us(5);
}

void i2c_stop() {
    I2C_DDR |= (1 << SDA_PIN);  // SDA as output
    _delay_us(5);
    I2C_PORT &= ~(1 << SCL_PIN);  // SCL low
    _delay_us(5);
    I2C_PORT |= (1 << SCL_PIN);   // SCL high
    _delay_us(5);
    I2C_PORT |= (1 << SDA_PIN);   // SDA high
    _delay_us(5);
}

void i2c_write(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            I2C_PORT |= (1 << SDA_PIN);
        } else {
            I2C_PORT &= ~(1 << SDA_PIN);
        }
        _delay_us(5);
        I2C_PORT |= (1 << SCL_PIN);
        _delay_us(5);
        I2C_PORT &= ~(1 << SCL_PIN);
        _delay_us(5);
        data <<= 1;
    }
    // Release SDA for ACK
    I2C_DDR &= ~(1 << SDA_PIN);
    _delay_us(5);
    I2C_PORT |= (1 << SCL_PIN);
    _delay_us(5);
    I2C_PORT &= ~(1 << SCL_PIN);
    _delay_us(5);
    I2C_DDR |= (1 << SDA_PIN);
}

void rtc_init() {
    i2c_start();
    i2c_write(PCF8523_ADDR << 1);  // Write mode
    i2c_write(0x00);  // Control_1 register
    i2c_write(0x00);  // Normal mode, 24-hour format
    i2c_stop();
}

uint8_t rtc_read_seconds() {
    i2c_start();
    i2c_write(PCF8523_ADDR << 1);  // Write mode
    i2c_write(0x03);  // Seconds register
    i2c_stop();
    
    i2c_start();
    i2c_write((PCF8523_ADDR << 1) | 1);  // Read mode
    // Read and return seconds
    uint8_t seconds = 0;
    for (uint8_t i = 0; i < 8; i++) {
        I2C_PORT |= (1 << SCL_PIN);
        _delay_us(5);
        if (I2C_PORT & (1 << SDA_PIN)) {
            seconds |= (1 << (7 - i));
        }
        I2C_PORT &= ~(1 << SCL_PIN);
        _delay_us(5);
    }
    i2c_stop();
    return seconds;
}

// Timer1 setup for PWM
void setup_timer1() {
    // Set Timer1 to Fast PWM mode, 8-bit
    TCCR1A |= (1 << WGM10) | (1 << COM1A0);  // Toggle OC1A on compare match
    TCCR1B |= (1 << WGM12) | (1 << CS10);     // No prescaler
    
    // Set initial frequency
    OCR1A = (F_CPU / (2 * BARK_FREQUENCY)) - 1;
}

void generate_tone(uint16_t frequency) {
    if (frequency == 0) {
        TCCR1A &= ~(1 << COM1A0);  // Disable PWM output
        SPEAKER_PORT &= ~(1 << SPEAKER_PIN);  // Ensure pin is low
    } else {
        TCCR1A |= (1 << COM1A0);   // Enable PWM output
        OCR1A = (F_CPU / (2 * frequency)) - 1;
    }
}

void generate_bark() {
    for (uint8_t i = 0; i < BARK_COUNT; i++) {
        // Start with higher frequency, decrease over time
        for (uint16_t freq = BARK_FREQUENCY; freq > BARK_FREQUENCY/2; freq -= 50) {
            generate_tone(freq);
            _delay_ms(BARK_DURATION/BARK_COUNT);
        }
        generate_tone(0);  // Stop sound
        _delay_ms(BARK_INTERVAL);
    }
}

void check_sleep_timer() {
    static uint8_t last_seconds = 0;
    static uint32_t inactive_seconds = 0;
    
    uint8_t current_seconds = rtc_read_seconds();
    
    // Check if a minute has passed
    if (current_seconds < last_seconds) {
        inactive_seconds += 60 - last_seconds + current_seconds;
    } else {
        inactive_seconds += current_seconds - last_seconds;
    }
    
    last_seconds = current_seconds;
    
    // Check if 24 hours have passed (86400 seconds)
    if (inactive_seconds >= 86400) {
        generate_bark();
        inactive_seconds = 0;  // Reset the counter
    }
}

int main(void) {
    // Initialize I2C and RTC
    i2c_init();
    rtc_init();
    
    // Set speaker pin as output
    SPEAKER_DDR |= (1 << SPEAKER_PIN);
    
    // Initialize timer for PWM
    setup_timer1();
    
    // For testing, generate a bark every 5 seconds
    while (1) {
        generate_bark();
        _delay_ms(5000);
        
        // Check sleep timer (commented out for testing)
        // check_sleep_timer();
    }
    
    return 0;
}
