#include "ruff_route.h"

// State variables
float current_latitude = 0.0;
float current_longitude = 0.0;
float target_latitude = 0.0;
float target_longitude = 0.0;
uint8_t current_speed = 0;
uint8_t current_direction = 0;
uint8_t obstacle_detected = 0;
uint8_t navigation_mode = 0;  // 0: manual, 1: autonomous

// Timer variables
volatile uint32_t gps_timer = 0;
volatile uint32_t display_timer = 0;
volatile uint32_t button_timer = 0;
volatile uint32_t seconds_counter = 0; // For 24h timer

// LCD display buffers
char lcd_buffer[4][21] = {
    "                    ",
    "                    ",
    "                    ",
    "                    "
};

// State machine
typedef enum {
    STATE_INITIAL,
    STATE_BARKING,
    STATE_ROUTE_SELECT,
    STATE_SPEED_SELECT,
    STATE_WALKING,
    STATE_CONGRATS
} RuffState;

RuffState current_state = STATE_INITIAL;
uint8_t selected_route = 0; // 0 or 1
uint8_t selected_speed = 1; // 1=slow, 2=med, 3=fast
uint8_t walk_complete = 0;

#define SECONDS_IN_24H (24*60*60)

// --- Route waypoints (single definition here, shared via extern) ---
const float route[] = {
    34.020392, -118.289929,
    34.020304, -118.289729,
    34.020245, -118.289596,
    34.020182, -118.289456,
    34.020161, -118.289413,
    34.020197, -118.289380,
    34.020157, -118.289401,
    34.020116, -118.289311,
};
const uint8_t route_size = sizeof(route)/(2*sizeof(float));

// --- Helper for miles conversion ---
#define METERS_TO_MILES 0.000621371f

// Initialize Timer1 for various timing tasks
void init_timer1(void) {
    // CTC mode, prescaler = 64
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    // Set compare value for 10ms intervals
    OCR1A = (F_CPU / 64 / 100) - 1;
    // Enable compare interrupt
    TIMSK1 = (1 << OCIE1A);
}

// Initialize buttons
void init_buttons(void) {
    // Set button pins as inputs with pull-ups
    DDRC &= ~((1 << LEFT_BUTTON) | (1 << SELECT_BUTTON) | (1 << RIGHT_BUTTON));
    PORTC |= (1 << LEFT_BUTTON) | (1 << SELECT_BUTTON) | (1 << RIGHT_BUTTON);
}

// Initialize LCD
void init_lcd(void) {
    // Initialize I2C
    i2c_init();
    // Initialize LCD
    lcd_init();
    // Clear display
    lcd_clear();
}

// Update LCD display
void update_display(void) {
    if (navigation_mode == 0) {  // Manual mode
        snprintf(lcd_buffer[0], 21, "Mode: Manual        ");
        snprintf(lcd_buffer[1], 21, "Speed: %d          ", current_speed);
        snprintf(lcd_buffer[2], 21, "Dist: %.1f cm      ", get_distance());
        snprintf(lcd_buffer[3], 21, "Press SEL for auto ");
    } else {  // Autonomous mode
        snprintf(lcd_buffer[0], 21, "Mode: Autonomous   ");
        snprintf(lcd_buffer[1], 21, "Lat: %.6f", current_latitude);
        snprintf(lcd_buffer[2], 21, "Lon: %.6f", current_longitude);
        snprintf(lcd_buffer[3], 21, "Dist: %.1f m       ", 
                 calculate_distance(current_latitude, current_longitude, 
                                  target_latitude, target_longitude));
    }
    lcd_screen(lcd_buffer);
}

// Play sound using Timer1
void play_sound(uint16_t frequency, uint16_t duration) {
    // Configure Timer1 for sound generation
    TCCR1A = (1 << COM1A0);  // Toggle OC1A on compare match
    TCCR1B = (1 << WGM12);   // CTC mode
    OCR1A = (F_CPU / (2 * frequency)) - 1;
    
    // Enable Timer1
    TCCR1B |= (1 << CS10);
    
    // Wait for duration
    _delay_ms(duration);
    
    // Stop sound
    TCCR1B &= ~(1 << CS10);
    PORTB &= ~(1 << SPEAKER_PIN);
}

// --- Helper functions for each state ---
void show_initial_screen(void) {
    snprintf(lcd_buffer[0], 21, "WELCOME           ");
    snprintf(lcd_buffer[1], 21, "Select your route ");
    snprintf(lcd_buffer[2], 21, "WOOF              ");
    snprintf(lcd_buffer[3], 21, "                  ");
    lcd_screen(lcd_buffer);
}

void show_barking_screen(void) {
    snprintf(lcd_buffer[0], 21, "TIME'S UP!        ");
    snprintf(lcd_buffer[1], 21, "Take me for a walk");
    snprintf(lcd_buffer[2], 21, "Press any button  ");
    snprintf(lcd_buffer[3], 21, "to stop barking   ");
    lcd_screen(lcd_buffer);
}

void show_route_select_screen(void) {
    snprintf(lcd_buffer[0], 21, "Select Route      ");
    snprintf(lcd_buffer[1], 21, selected_route == 0 ? "> ROUTE 1         " : "  ROUTE 1         ");
    snprintf(lcd_buffer[2], 21, selected_route == 1 ? "> ROUTE 2         " : "  ROUTE 2         ");
    snprintf(lcd_buffer[3], 21, "Press SEL to pick ");
    lcd_screen(lcd_buffer);
}

void show_speed_select_screen(void) {
    snprintf(lcd_buffer[0], 21, "Speed:            ");
    snprintf(lcd_buffer[1], 21, selected_speed == 1 ? "> slow   medium fast" : "  slow   medium fast");
    snprintf(lcd_buffer[2], 21, selected_speed == 2 ? "> slow   medium fast" : "  slow   medium fast");
    snprintf(lcd_buffer[3], 21, selected_speed == 3 ? "> slow   medium fast" : "  slow   medium fast");
    lcd_screen(lcd_buffer);
}

void show_walking_screen(float miles_walked) {
    snprintf(lcd_buffer[0], 21, "LETS GO!          ");
    snprintf(lcd_buffer[1], 21, "Miles walked: %.2f ", miles_walked);
    snprintf(lcd_buffer[2], 21, "                  ");
    snprintf(lcd_buffer[3], 21, "                  ");
    lcd_screen(lcd_buffer);
}

void show_congrats_screen(void) {
    snprintf(lcd_buffer[0], 21, "Route finished    ");
    snprintf(lcd_buffer[1], 21, "-- CONGRATS --    ");
    snprintf(lcd_buffer[2], 21, "                  ");
    snprintf(lcd_buffer[3], 21, "                  ");
    lcd_screen(lcd_buffer);
}

// --- Main ---
int main(void) {
    // Initialize all peripherals
    init_gpio();
    init_pwm();
    init_uart();
    init_timer1();
    init_ultrasonic();
    init_lcd();
    init_buttons();
    sei();

    uint32_t congrats_timer = 0;
    float miles_walked = 0.0;
    uint8_t barking = 0;
    uint8_t current_wp = 0;
    float last_lat = 0.0, last_lon = 0.0;
    uint8_t gps_initialized = 0;

    show_initial_screen();

    while (1) {
        switch (current_state) {
            case STATE_INITIAL:
                show_initial_screen();
                stop_motors();
                // Wait for 24h or button press
                if (seconds_counter >= SECONDS_IN_24H) {
                    current_state = STATE_BARKING;
                    barking = 1;
                } else if (button_pressed()) {
                    current_state = STATE_ROUTE_SELECT;
                    seconds_counter = 0;
                }
                break;
            case STATE_BARKING:
                show_barking_screen();
                play_sound(1000, 200); // Barking sound
                if (button_pressed()) {
                    barking = 0;
                    current_state = STATE_ROUTE_SELECT;
                    seconds_counter = 0;
                }
                break;
            case STATE_ROUTE_SELECT:
                show_route_select_screen();
                // Handle left/right/select buttons
                if (left_button_pressed()) {
                    selected_route = (selected_route == 0) ? 1 : 0;
                } else if (right_button_pressed()) {
                    selected_route = (selected_route == 1) ? 0 : 1;
                } else if (select_button_pressed()) {
                    current_state = STATE_SPEED_SELECT;
                }
                break;
            case STATE_SPEED_SELECT:
                show_speed_select_screen();
                if (left_button_pressed()) {
                    selected_speed = (selected_speed == 1) ? 3 : selected_speed - 1;
                } else if (right_button_pressed()) {
                    selected_speed = (selected_speed == 3) ? 1 : selected_speed + 1;
                } else if (select_button_pressed()) {
                    current_state = STATE_WALKING;
                    // Set PWM based on speed
                    if (selected_speed == 1) current_speed = 43;
                    else if (selected_speed == 2) current_speed = 86;
                    else current_speed = 128;
                    forward(current_speed);
                }
                break;
            case STATE_WALKING:
                // Update GPS position periodically
                if (gps_timer >= GPS_UPDATE_INTERVAL) {
                    update_gps();
                    gps_timer = 0;
                }
                // On first valid GPS, set last_lat/lon
                if (!gps_initialized && current_latitude != 0.0 && current_longitude != 0.0) {
                    last_lat = current_latitude;
                    last_lon = current_longitude;
                    gps_initialized = 1;
                }
                // Track distance walked
                if (gps_initialized) {
                    float d = calculate_distance(last_lat, last_lon, current_latitude, current_longitude);
                    if (d > 1.0) { // Only count if moved >1m
                        miles_walked += d * METERS_TO_MILES;
                        last_lat = current_latitude;
                        last_lon = current_longitude;
                    }
                }
                // Obstacle avoidance
                float distance = get_distance();
                if (distance < STOP_DISTANCE) {
                    stop_motors();
                } else {
                    forward(current_speed);
                }
                // Check if reached current waypoint
                float wp_lat = route[2*current_wp];
                float wp_lon = route[2*current_wp+1];
                float dist_to_wp = calculate_distance(current_latitude, current_longitude, wp_lat, wp_lon);
                if (dist_to_wp < 15.0) { // 15m threshold
                    if (++current_wp >= route_size) {
                        walk_complete = 1;
                    }
                }
                show_walking_screen(miles_walked);
                if (walk_complete) {
                    current_state = STATE_CONGRATS;
                    congrats_timer = 0;
                }
                break;
            case STATE_CONGRATS:
                show_congrats_screen();
                stop_motors();
                congrats_timer++;
                if (congrats_timer > 3000) { // ~30s if loop runs every 10ms
                    current_state = STATE_INITIAL;
                    seconds_counter = 0;
                    walk_complete = 0;
                }
                break;
        }
        _delay_ms(10);
    }
    return 0;
}

// --- Timer1 interrupt for timing various tasks ---
ISR(TIMER1_COMPA_vect) {
    gps_timer += 10;
    display_timer += 10;
    button_timer += 10;
    static uint16_t ms_counter = 0;
    ms_counter += 10;
    if (ms_counter >= 1000) {
        seconds_counter++;
        ms_counter = 0;
    }
}

// --- Button helpers (debounced, one-shot detection) ---
uint8_t button_pressed(void) {
    static uint8_t last_state = 0xFF;
    static uint32_t last_time = 0;
    uint8_t state = (PINC & ((1 << LEFT_BUTTON) | (1 << SELECT_BUTTON) | (1 << RIGHT_BUTTON)));
    if (state != last_state && (button_timer - last_time) > BUTTON_DEBOUNCE_TIME) {
        last_time = button_timer;
        last_state = state;
        if ((state & (1 << LEFT_BUTTON)) == 0 || (state & (1 << SELECT_BUTTON)) == 0 || (state & (1 << RIGHT_BUTTON)) == 0) {
            return 1;
        }
    }
    return 0;
}

uint8_t left_button_pressed(void) {
    static uint8_t last_state = 0xFF;
    static uint32_t last_time = 0;
    uint8_t state = (PINC & (1 << LEFT_BUTTON));
    if (state != last_state && (button_timer - last_time) > BUTTON_DEBOUNCE_TIME) {
        last_time = button_timer;
        last_state = state;
        if ((state & (1 << LEFT_BUTTON)) == 0) {
            return 1;
        }
    }
    return 0;
}

uint8_t right_button_pressed(void) {
    static uint8_t last_state = 0xFF;
    static uint32_t last_time = 0;
    uint8_t state = (PINC & (1 << RIGHT_BUTTON));
    if (state != last_state && (button_timer - last_time) > BUTTON_DEBOUNCE_TIME) {
        last_time = button_timer;
        last_state = state;
        if ((state & (1 << RIGHT_BUTTON)) == 0) {
            return 1;
        }
    }
    return 0;
}

uint8_t select_button_pressed(void) {
    static uint8_t last_state = 0xFF;
    static uint32_t last_time = 0;
    uint8_t state = (PINC & (1 << SELECT_BUTTON));
    if (state != last_state && (button_timer - last_time) > BUTTON_DEBOUNCE_TIME) {
        last_time = button_timer;
        last_state = state;
        if ((state & (1 << SELECT_BUTTON)) == 0) {
            return 1;
        }
    }
    return 0;
}

// Navigation function
void navigate_to_waypoint(float target_lat, float target_lon) {
    if (obstacle_detected) {
        return;  // Don't navigate if obstacle detected
    }
    
    float bearing = calculate_bearing(current_latitude, current_longitude, target_lat, target_lon);
    float distance = calculate_distance(current_latitude, current_longitude, target_lat, target_lon);
    
    // If we're close to the target, stop
    if (distance < 1.0) {  // Within 1 meter
        stop_motors();
        return;
    }
    
    // Adjust direction based on bearing
    if (bearing > 10.0) {
        turn_right(TURN_SPEED);
    } else if (bearing < -10.0) {
        turn_left(TURN_SPEED);
    } else {
        forward(FORWARD_SPEED);
    }
}

// Obstacle avoidance
void avoid_obstacle(void) {
    // Back up slightly
    reverse(REVERSE_SPEED);
    _delay_ms(500);
    
    // Turn right
    turn_right(TURN_SPEED);
    _delay_ms(1000);
    
    // Check if path is clear
    float distance = get_distance();
    if (distance > STOP_DISTANCE) {
        forward(FORWARD_SPEED);
    } else {
        // If still blocked, try turning left
        turn_left(TURN_SPEED);
        _delay_ms(2000);
        
        distance = get_distance();
        if (distance > STOP_DISTANCE) {
            forward(FORWARD_SPEED);
        } else {
            // If still blocked, reverse and try again
            reverse(REVERSE_SPEED);
            _delay_ms(1000);
        }
    }
}

// Button handling
void handle_buttons(void) {
    static uint8_t last_button_state = 0;
    static uint32_t last_debounce_time = 0;
    
    uint8_t button_state = (PINC & ((1 << LEFT_BUTTON) | (1 << SELECT_BUTTON) | (1 << RIGHT_BUTTON)));
    
    if (button_state != last_button_state) {
        last_debounce_time = button_timer;
    }
    
    if ((button_timer - last_debounce_time) > BUTTON_DEBOUNCE_TIME) {
        if (button_state != last_button_state) {
            last_button_state = button_state;
            
            if (!(PINC & (1 << LEFT_BUTTON))) {
                turn_left(TURN_SPEED);
            } else if (!(PINC & (1 << RIGHT_BUTTON))) {
                turn_right(TURN_SPEED);
            } else if (!(PINC & (1 << SELECT_BUTTON))) {
                // Toggle navigation mode
                navigation_mode = !navigation_mode;
                if (navigation_mode) {
                    play_sound(2000, 100);  // High tone for autonomous mode
                } else {
                    play_sound(1000, 100);  // Low tone for manual mode
                }
            }
        }
    }
} 