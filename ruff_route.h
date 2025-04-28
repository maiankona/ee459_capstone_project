#ifndef RUFF_ROUTE_H
#define RUFF_ROUTE_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Pin Definitions
// Speaker
#define SPEAKER_PIN PB1

// GPS Module (UART)
#define GPS_RX PD0
#define GPS_TX PD1

// Motors
#define MOTOR_A_DIR1 PD4
#define MOTOR_A_DIR2 PD5
#define MOTOR_B_DIR1 PD6
#define MOTOR_B_DIR2 PD7
#define MOTOR_PWM PB2

// Ultrasonic Sensor
#define TRIG_PIN PB7
#define ECHO_PIN PD2

// Buttons
#define LEFT_BUTTON PC1
#define SELECT_BUTTON PC2
#define RIGHT_BUTTON PC3

// LCD Screen (I2C)
#define LCD_SDA PC4
#define LCD_SCL PC5
#define LCD_CS PD3

// Constants
#define MAX_SPEED 255
#define MIN_SPEED 0
#define TURN_SPEED 150
#define FORWARD_SPEED 200
#define REVERSE_SPEED 150
#define STOP_DISTANCE 30  // cm
#define SLOW_DOWN_DISTANCE 50  // cm
#define GPS_UPDATE_INTERVAL 1000  // ms
#define BUTTON_DEBOUNCE_TIME 50  // ms

// Function Declarations
void init_gpio(void);
void init_pwm(void);
void init_uart(void);
void init_timer1(void);
void init_ultrasonic(void);
void init_lcd(void);
void init_buttons(void);

void motor_control(uint8_t motor, uint8_t direction, uint8_t speed);
void set_motor_speed(uint8_t motor, uint8_t speed);
void stop_motors(void);
void forward(uint8_t speed);
void reverse(uint8_t speed);
void turn_left(uint8_t speed);
void turn_right(uint8_t speed);

float get_distance(void);
void update_gps(void);
void update_display(void);
void handle_buttons(void);
void play_sound(uint16_t frequency, uint16_t duration);

// Navigation functions
void navigate_to_waypoint(float target_lat, float target_lon);
float calculate_bearing(float lat1, float lon1, float lat2, float lon2);
float calculate_distance(float lat1, float lon1, float lat2, float lon2);
void avoid_obstacle(void);

// State variables
extern float current_latitude;
extern float current_longitude;
extern float target_latitude;
extern float target_longitude;
extern uint8_t current_speed;
extern uint8_t current_direction;
extern uint8_t obstacle_detected;
extern uint8_t navigation_mode;

extern const float route[];
extern const uint8_t route_size;

#endif // RUFF_ROUTE_H 