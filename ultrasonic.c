#define F_CPU 7372800UL
#include <avr/io.h>
#include <util/delay.h>

// ───── Ultrasonic ────────────────────────────────────────────────────────────
#define TRIG_DDR   DDRC
#define TRIG_PORT  PORTC
#define TRIG_PIN   PC7
#define ECHO_DDR   DDRD
#define ECHO_PIN   PD2
#define ECHO_PIN_REG PIND

#define WARNING_DISTANCE  50
#define CRITICAL_DISTANCE 20
#define MAX_DISTANCE     400
#define MIN_DISTANCE       2

#define NUM_READINGS      5
#define READING_INTERVAL 20
#define SMOOTHING_WINDOW  5

uint16_t distance_history[SMOOTHING_WINDOW];
uint8_t  history_index     = 0;
uint16_t last_valid_distance = MAX_DISTANCE;
uint8_t  error_count       = 0;
#define MAX_ERRORS 3

void setup_timer1_ultra(void) {
    // Timer1 free-running, prescaler=1
    TCCR1B = (1<<CS10);
}

void send_trigger(void) {
    TRIG_PORT |=  (1<<TRIG_PIN);
    _delay_us(10);
    TRIG_PORT &= ~(1<<TRIG_PIN);
}

uint16_t get_single_reading(void) {
    // pull TRIG low, wait
    TRIG_PORT &= ~(1<<TRIG_PIN);
    _delay_us(2);

    // clear and fire
    TCNT1 = 0;
    send_trigger();

    // wait for echo high
    while (!(ECHO_PIN_REG & (1<<ECHO_PIN)));
    TCNT1 = 0;

    // wait for echo low or timeout
    while ((ECHO_PIN_REG & (1<<ECHO_PIN)) && (TCNT1 < 60000));

    // convert ticks to cm: ticks * 17 /1000
    uint16_t d = (TCNT1 * 17UL) / 1000UL;
    return (d>=MIN_DISTANCE && d<=MAX_DISTANCE) ? d : 0;
}

uint16_t get_distance(void) {
    uint16_t readings[NUM_READINGS];
    uint8_t  valid=0;

    for (uint8_t i=0; i<NUM_READINGS; i++) {
        uint16_t r = get_single_reading();
        if (r) readings[valid++] = r;
        _delay_ms(READING_INTERVAL);
    }

    if (!valid) {
        if (++error_count >= MAX_ERRORS) return last_valid_distance;
        return 0;
    }

    // median
    for (uint8_t i=0; i<valid-1; i++)
        for (uint8_t j=i+1; j<valid; j++)
            if (readings[j]<readings[i]) {
                uint16_t t=readings[i];
                readings[i]=readings[j];
                readings[j]=t;
            }
    uint16_t med = readings[valid/2];

    // rolling average
    distance_history[history_index] = med;
    history_index = (history_index+1)%SMOOTHING_WINDOW;
    uint32_t sum=0;
    for (uint8_t i=0; i<SMOOTHING_WINDOW; i++) sum+=distance_history[i];
    last_valid_distance = sum/SMOOTHING_WINDOW;
    error_count=0;
    return last_valid_distance;
}

// ───── Motor A (Front Left) on OC1B/PB2 ──────────────────────────────────────
#define MOTOR_A_PIN1  PD6
#define MOTOR_A_PIN2  PD7
#define MOTOR_A_PWM   PB2  // OC1B

// ───── Motor B (Front Right) on OC2B/PD3 ───────────────────────────────────
#define MOTOR_B_PIN1  PD4
#define MOTOR_B_PIN2  PD5
#define MOTOR_B_PWM   PD3  // OC2B

void init_motors(void) {
    // direction pins
    DDRD |= (1<<MOTOR_A_PIN1)|(1<<MOTOR_A_PIN2)
         | (1<<MOTOR_B_PIN1)|(1<<MOTOR_B_PIN2);
}

void init_pwm(void) {
    // Motor A: Fast PWM, 8-bit, non-invert on OC1B, prescaler=8
    DDRB |= (1<<MOTOR_A_PWM);
    TCCR1A = (1<<COM1B1)|(1<<WGM10);
    TCCR1B = (1<<WGM12)|(1<<CS11);
    OCR1B = 0;

    // Motor B: Fast PWM, 8-bit, non-invert on OC2B, prescaler=8
    DDRD |= (1<<MOTOR_B_PWM);
    TCCR2A = (1<<COM2B1)|(1<<WGM21)|(1<<WGM20);
    TCCR2B = (1<<CS21);
    OCR2B = 0;
}

static inline void set_motor_a(int16_t speed) {
    if (speed>0) {
        PORTD |=  (1<<MOTOR_A_PIN1);
        PORTD &= ~(1<<MOTOR_A_PIN2);
    } else if (speed<0) {
        PORTD &= ~(1<<MOTOR_A_PIN1);
        PORTD |=  (1<<MOTOR_A_PIN2);
        speed = -speed;
    } else {
        PORTD &= ~((1<<MOTOR_A_PIN1)|(1<<MOTOR_A_PIN2));
    }
    if (speed>255) speed=255;
    OCR1B = speed;
}

static inline void set_motor_b(int16_t speed) {
    if (speed>0) {
        PORTD |=  (1<<MOTOR_B_PIN1);
        PORTD &= ~(1<<MOTOR_B_PIN2);
    } else if (speed<0) {
        PORTD &= ~(1<<MOTOR_B_PIN1);
        PORTD |=  (1<<MOTOR_B_PIN2);
        speed = -speed;
    } else {
        PORTD &= ~((1<<MOTOR_B_PIN1)|(1<<MOTOR_B_PIN2));
    }
    if (speed>255) speed=255;
    OCR2B = speed;
}

// ───── Obstacle check ────────────────────────────────────────────────────────
void check_obstacle_and_drive(void) {
    uint16_t d = get_distance();
    if (!d) return; // bad reading: leave last state

    if (d <= CRITICAL_DISTANCE) {
        // STOP
        set_motor_a(0);
        set_motor_b(0);
    }
    else if (d <= WARNING_DISTANCE) {
        // SLOW FORWARD at half-speed
        set_motor_a(+100);
        set_motor_b(+100);
    }
    else {
        // CLEAR: FULL-SPEED FORWARD
        set_motor_a(+200);
        set_motor_b(+200);
    }
}

int main(void) {
    // pins
    TRIG_DDR |=  (1<<TRIG_PIN);
    ECHO_DDR &= ~(1<<ECHO_PIN);

    init_motors();
    init_pwm();
    setup_timer1_ultra();

    // init history
    for (uint8_t i=0;i<SMOOTHING_WINDOW;i++)
        distance_history[i]=MAX_DISTANCE;

    while (1) {
        check_obstacle_and_drive();
        _delay_ms(50);
    }
}
