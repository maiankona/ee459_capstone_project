/* Initial state
1. turn all motors off : // Motor A (Front Left) direction pins
#define MOTOR_A_PIN1   PD6
#define MOTOR_A_PIN2   PD7
#define MOTOR_A_PWM    PB2

// Motor B (Front Right) direction pins
#define MOTOR_B_PIN1   PD4
#define MOTOR_B_PIN2   PD5
#define MOTOR_B_PWM    PB2

2. turn led lights on by switching power on to the board
3. route selection is cleared (only 2 routes)
4. start to keep track of time and mark when 24 hours hits and speaker outputs beep sound (from bark.c file) when time is up until user turns on the dog
5. LCD screen displays text "WELCOME
                            Select your route
                            WOOF"
20x4 characters, serial (I2C), white-on-blue Crystalfontz CFAH2004AC-TMI-EW lcd we're using sda and scl pins from atmegat328p, csb pin will be PD3
arrow state transitions:
- if no button is clicked stay in intial state
- if any button PC1 (left), PC2 (select), PC3 (right) is clicked go to state route 1
*/

/*
Route 1 state
1. LCD displays "Route 1" and "Route 2" below
- if left or right button is clicked go to state Route 2
- if select button button is clicked go to slow state 
*/

/*
Route 2 state
1. LCD displays "Route 1" and "Route 2" below
- if left or right button is clicked go to state Route 1
- if select button button is clicked go to slow state 
*/

/*
Slow state 
1. LCD displays "Speed: **slow**  medium  fast"
2. speed = 1 -> PWM goes to 43
- if select is pressed -> Lets Go state
- if left is pressed (wraps around) -> Fast state
- if right is pressed -> Medium state
- if no button pressed stay in slow state
*/

/*
Medium state
1. LCD displays "Speed: slow  **medium**  fast"
2. speed = 2 -> PWM goes to 86
- if select is pressed -> Lets Go state
- if left is pressed -> Slow state
- if right is pressed -> Fast state
- if no button pressed stay in medium state
*/

/*
Fast state
1. LCD displays "Speed: slow  medium  **fast**"
2. speed = 3 -> PWM goes to 128
- if select is pressed -> Lets Go state
- if left is pressed -> Medium state
- if right is pressed (wrap around) -> Slow state
- if no button pressed stay in fast state
*/

/*
Lets Go state
1. LCD displays "LETS GO!
                miles walked: 00 "
2. motors go on until the route is finished via geofencing.c file
- geofencing code should also include ultrasonic sensor code that when an object is in front of the dog, the motors stop until he object goes away, then it continues the route
3. keeps track of distance via GPS module coordinates and updates it every second on the lcd
GPS uses UART pins
4. when the path is finished (last coordinate on gps module route is reached) -> go to Congrats state
*/

/*
Congrats state
1. LCD display "Route finished-- CONGRATS"
- display this for 30 seconds and immediately go to initial state
*/
