/*
 * Basic Motor Control on ATmega328P (Bare-metal AVR C)
 * This code cycles through four motor-driving patterns,
 * each running for 5 seconds, by directly setting the PORT registers.
 *
 * Assumptions:
 *  - The ATmega328P is clocked at 16 MHz.
 *  - Four motor control signals are connected to PORTD pins:
 *       Motor A, Direction Pin 1: PD2
 *       Motor A, Direction Pin 2: PD3
 *       Motor B, Direction Pin 1: PD4
 *       Motor B, Direction Pin 2: PD5
 *
 * To compile and load this code, use an AVR-GCC toolchain.
 */

 #define F_CPU 16000000UL    // Define CPU frequency for delay calculations

 #include <avr/io.h>
 #include <util/delay.h>
 #include <stdint.h>
 
 // Define bit masks for motor control pins
 #define MOTOR_A_PIN1 (1 << PD2)  // Motor A direction 1 (e.g., forward)
 #define MOTOR_A_PIN2 (1 << PD3)  // Motor A direction 2 (e.g., reverse)
 #define MOTOR_B_PIN1 (1 << PD4)  // Motor B direction 1 (e.g., forward)
 #define MOTOR_B_PIN2 (1 << PD5)  // Motor B direction 2 (e.g., reverse)
 
 // Initialize the motor control pins as outputs
 void init_motor_pins(void)
 {
     // DDRD controls the data direction for Port D.
     // Setting bits corresponding to our motor pins to 1 makes them outputs.
     DDRD |= MOTOR_A_PIN1 | MOTOR_A_PIN2 | MOTOR_B_PIN1 | MOTOR_B_PIN2;
 }
 
 // Helper function to set motor A outputs
 // state1 corresponds to MOTOR_A_PIN1 and state2 to MOTOR_A_PIN2.
 void set_motor_a(uint8_t state1, uint8_t state2)
 {
     if (state1)
         PORTD |= MOTOR_A_PIN1;
     else
         PORTD &= ~MOTOR_A_PIN1;
     
     if (state2)
         PORTD |= MOTOR_A_PIN2;
     else
         PORTD &= ~MOTOR_A_PIN2;
 }
 
 // Helper function to set motor B outputs
 // state1 corresponds to MOTOR_B_PIN1 and state2 to MOTOR_B_PIN2.
 void set_motor_b(uint8_t state1, uint8_t state2)
 {
     if (state1)
         PORTD |= MOTOR_B_PIN1;
     else
         PORTD &= ~MOTOR_B_PIN1;
     
     if (state2)
         PORTD |= MOTOR_B_PIN2;
     else
         PORTD &= ~MOTOR_B_PIN2;
 }
 
 int main(void)
 {
     // Initialize motor control pins
     init_motor_pins();
 
     // Main loop - cycle through motor control patterns indefinitely
     while (1)
     {
         // Pattern 1: Both motors forward (motor: PIN1 HIGH, PIN2 LOW)
         set_motor_a(1, 0);
         set_motor_b(1, 0);
         _delay_ms(5000);  // Wait 5 seconds
 
         // Pattern 2: Both motors reverse (motor: PIN1 LOW, PIN2 HIGH)
         set_motor_a(0, 1);
         set_motor_b(0, 1);
         _delay_ms(5000);  // Wait 5 seconds
 
         // Pattern 3: Motor A forward, Motor B reverse
         set_motor_a(1, 0);
         set_motor_b(0, 1);
         _delay_ms(5000);  // Wait 5 seconds
 
         // Pattern 4: Motor A reverse, Motor B forward
         set_motor_a(0, 1);
         set_motor_b(1, 0);
         _delay_ms(5000);  // Wait 5 seconds
 
     }
 
     // This return statement is never reached, but is included for completeness.
     return 0;
 }
 