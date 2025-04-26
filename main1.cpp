
// LIBRARIES ETC ------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <Adafruit_GFX.h> // INSTALL THIS
#include <SPI.h> // This comes with the above library
#include <Adafruit_ILI9341.h> // This comes with the above library
#include <trivia.h> // INCLUDE THIS FILE
#include <melody_player.h> // INCLUDE THIS AFTER INSTALLING Melody Player by Fabiano Riccardi
#include <melody_factory.h> // INCLUDE THIS AFTER INSTALLING Melody Player by Fabiano Riccardi
#include <Servo.h> // INCLUDE THIS AFTER INSTALLING ESP32 ESP32S2 AnalogWrite by David Lloyd
#include <PubSubClient.h>
#include <WiFi.h>
#include <Adafruit_I2CDevice.h>
#include <SIKTEC_SRAM.h>
#include <SdFat.h>
#include <Adafruit_GFX.h>


// PINS ---------------------------------------------------------------------------------------------------
#define TFT_CS   21
#define TFT_DC   17
#define pin_button_LEFT 39
#define pin_button_RIGHT 36
Servo servo = Servo();
#define pin_servo_head 26
// #define pin_servo_leftarm 25
// #define pin_servo_rightarm 34
#define pin_buzzer1 4
// #define pin_buzzer2 36
// #define pin_button_MENU 4
// Defining pins for motor drive
const int PIN_AIN2 = 15; // pins to control motor A
const int PIN_AIN1 = 33;
const int PIN_BIN1 = 27; // pins to control motor B
const int PIN_BIN2 = 12;

const char* ssid = "ESP32AP";  // sets up network
const char* password = "Password";
const char* serverIP = "192.168.4.1"; // IP address of the ESP32
int BlueButton = 1; // sets initial values for buttons (0 when clicked)
int RedButton = 1;
int GreenButton = 1;
int YellowButton = 1;

WiFiServer server(80); // creates wifi server
WiFiClient client;


// SOUND SETUP --------------------------------------------------------------------------------------------
const int nNotes = 4;
String notes_correct1[nNotes] = { "E5", "G5", "C6", "SILENCE"};
String notes_correct2[nNotes] = { "G5", "C6", "E6", "SILENCE"};
String notes_incorrect1[nNotes] = { "C5", "B4", "A4", "SILENCE"};
String notes_incorrect2[nNotes] = { "E5", "D5", "C5", "SILENCE"};
const int timeUnit = 100;
Melody melody_correct1 = MelodyFactory.load("Nice Melody", timeUnit, notes_correct1, nNotes);
// Melody melody_correct2 = MelodyFactory.load("Nice Melody", timeUnit, notes_correct2, nNotes);
Melody melody_incorrect1 = MelodyFactory.load("Nice Melody", timeUnit, notes_incorrect1, nNotes);
// Melody melody_incorrect2 = MelodyFactory.load("Nice Melody", timeUnit, notes_incorrect2, nNotes);
MelodyPlayer player1(pin_buzzer1);
// MelodyPlayer player2(pin_buzzer2, 2);

// GAME SETUP ----------------------------------------------------------------------------------------------
int highscore_math = 0;
int highscore_trivia = 0;
int currbuttonval_LEFT = LOW;
int currbuttonval_RIGHT = LOW;
// int currbuttonval_MENU = HIGH;
int prevbuttonval_LEFT = LOW;
int prevbuttonval_RIGHT = LOW;
// int prevbuttonval_MENU = HIGH;
int math_ans = 1;
String problem = "";
int answer = 0;
String question_trivia = "";
String answer_trivia = "";
String explained = "";
int ans_leftorright = 0;
int curr_score = 0;
unsigned long currMillis = 0;
unsigned long prevMillis = 0;

enum Screen {Home, Math, Trivia};
Screen currentScreen = Home;

String division_questions[] = {"20 / 20", "20 / 10", "20 / 5", "20 / 4", "20 / 2", "20 / 1",
  "19 / 19", "19 / 1", "18 / 18", "18 / 9", "18 / 6", "18 / 3", "18 / 2", "18 / 1", "17 / 17", "17 / 1",
  "16 / 16", "16 / 8", "16 / 4", "16 / 2", "16 / 1", "15 / 15", "15 / 5", "15 / 3", "15 / 1", 
  "14 / 14", "14 / 7", "14 / 2", "14 / 1", "13 / 13", "13 / 1", "12 / 12", "12 / 6", "12 / 4", "12 / 3", "12 / 2", "12 / 1",
  "11 / 11", "11 / 1", "10 / 10", "10 / 5", "10 / 2", "10 / 1", "9 / 9", "9 / 3", "9 / 1", "8 / 8", "8 / 4", "8 / 2", "8 / 1", 
  "7 / 7", "7 / 1", "6 / 6", "6 / 3", "6 / 2", "6 / 1", "5 / 5", "5 / 1", "4 / 4", "4 / 2", "4 / 1", 
  "3 / 3", "3 / 1", "2 / 2", "2 / 1", "1 / 1"};

int division_answers[] = {1, 2, 4, 5, 10, 20, 1, 19, 1, 2, 3, 6, 9, 18, 1, 17, 1, 2, 4, 8, 16, 1, 3, 5, 15, 
  1, 2, 7, 14, 1, 13, 1, 2, 3, 4, 6, 12, 1, 11, 1, 2, 5, 10, 1, 3, 9, 1, 2, 4, 8, 1, 7, 1, 2, 3, 6, 1, 5, 
  1, 2, 4, 1, 3, 1, 2, 1};


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);


void homeScreen() { 
  tft.setTextColor(ILI9341_PURPLE);  tft.setTextSize(5);
  tft.setCursor(tft.width()/14, tft.height()/4);
  tft.println("ROBUDDY");

  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
  tft.setCursor(tft.width()/6, tft.height()*2/5);
  tft.println("Select game...");

  tft.drawRect(19, 183, 80, 40, ILI9341_RED);
  tft.setTextColor(ILI9341_RED);  tft.setTextSize(3);
  tft.setCursor(25, tft.height()*3/5);
  tft.println("MATH");

  tft.drawRect(115, 183, 115, 40, ILI9341_BLUE);
  tft.setTextColor(ILI9341_BLUE);
  tft.setCursor(119, tft.height()*3/5);
  tft.println("TRIVIA");

  tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(1);
  tft.setCursor(28, tft.height()*3/4);
  tft.println("HIGH SCORE");
  tft.setCursor(45, tft.height()*4/5);
  tft.println(highscore_math);
  tft.setCursor(tft.width()*8/13, tft.height()*3/4);
  tft.println("HIGH SCORE");
  tft.setCursor(tft.width()*9/13, tft.height()*4/5);
  tft.println(highscore_trivia);

}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_RED);
  tft.fillScreen(ILI9341_GREEN);
  tft.fillScreen(ILI9341_BLUE);
  tft.fillScreen(ILI9341_BLACK);
  homeScreen();
  return micros() - start;
}

void mathGame() {
  int problem_type = rand() % 4;
  int otherans = 0;
  if (problem_type == 0) { // DIVISION
    int rand_div = rand() % 66;
    problem = division_questions[rand_div];
    answer = division_answers[rand_div]; 
    otherans = rand() % 21;
    if (answer == otherans) {
      otherans += 1;
    }
  } else if (problem_type == 1) { // SUBTRACTION
    int rand1 = rand() % 11;
    int rand2 = rand() % 11;
    int greaternum = rand1;
    int smallernum = rand2;
    if (rand2 > rand1) {     
      greaternum = rand2;
      smallernum = rand1; }
    problem = String(greaternum) + " - " + String(smallernum);
    answer = greaternum - smallernum;
    otherans = rand() % 11;
    if (answer == otherans) {
      otherans += 1;
    }
  } else if (problem_type == 2) { // ADDITION
    int rand1 = rand() % 11;
    int rand2 = rand() % 11;
    problem = String(rand1) + " + " + String(rand2);
    answer = rand1 + rand2;
    otherans = rand() % 21;
    if (answer == otherans) {
      otherans += 1;
    }
  } else if (problem_type == 3) { // MULTIPLICATION
    int rand1 = rand() % 11;
    int rand2 = rand() % 11;
    problem = String(rand1) + " x " + String(rand2);
    answer = rand1 * rand2;
    otherans = rand() % 51;
    if (answer == otherans) {
      otherans += 1;
    }
  }

  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2.5);
  tft.setCursor(tft.width()/4, tft.height()/6);
  tft.println("Choose the");
  tft.setCursor(tft.width()/7, tft.height()/4);
  tft.println("correct answer!");

  tft.setTextColor(ILI9341_PURPLE);  tft.setTextSize(4);
  tft.setCursor(tft.width()/4, tft.height()*2/5);
  tft.println(problem);

  ans_leftorright = rand() % 2;

  tft.drawRect(28, 183, 80, 40, ILI9341_RED);
  tft.setTextColor(ILI9341_RED);  tft.setTextSize(3);
  tft.setCursor(tft.width()/6, tft.height()*3/5);
  if (ans_leftorright == 0) {
    tft.println(answer);
  } else {
    tft.println(otherans);
  }

  tft.drawRect(128, 183, 80, 40, ILI9341_BLUE);
  tft.setTextColor(ILI9341_BLUE);
  tft.setCursor(tft.width()*5/8, tft.height()*3/5);
  if (ans_leftorright == 1) {
    tft.println(answer);
  } else {
    tft.println(otherans);
  }
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(tft.width()/5, tft.height()*9/10);
  tft.println("Score: " + String(curr_score));

}

void triviaGame() {
  int rand_q = rand() % 68;
  question_trivia = trivia_questions[rand_q];
  answer_trivia = trivia_answers[rand_q];
  explained = trivia_answers_explained[rand_q];
  if (answer_trivia == "True") {
    ans_leftorright = 0;
  } else {ans_leftorright = 1;}
  
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2.5);
  tft.setCursor(tft.width()/4, tft.height()/6);
  tft.println("Choose the");
  tft.setCursor(tft.width()/7, tft.height()/4);
  tft.println("correct answer!");

  tft.setTextColor(ILI9341_PURPLE);  tft.setTextSize(2);
  tft.setCursor(0, tft.height()*2/5);
  tft.println(question_trivia);

  tft.drawRect(22, 210, 80, 40, ILI9341_RED);
  tft.setTextColor(ILI9341_RED);  tft.setTextSize(3);
  tft.setCursor(27, 220);
  tft.println("TRUE");

  tft.drawRect(120, 210, 100, 40, ILI9341_BLUE);
  tft.setTextColor(ILI9341_BLUE);
  tft.setCursor(127, 220);
  tft.println("FALSE");

  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(tft.width()/5, tft.height()*9/10);
  tft.println("Score: " + String(curr_score));

}

void correctMelody() {
  player1.playAsync(melody_correct1);
  // player2.playAsync(melody_correct2);
}

void incorrectMelody() {
  player1.playAsync(melody_incorrect1);
  // player2.playAsync(melody_incorrect2);
}

void moveHead() {
  servo.write(pin_servo_head, 180);
  delay(500);
  servo.write(pin_servo_head, 90);
  delay(500);
  // for (int pos = 90; pos <= 165; pos=pos+15) {  // go from 0-180 degrees
  //   servo.write(pin_servo_head, pos);        // set the servo position (degrees)
  //   delay(20);
  // }
  // for (int pos = 165; pos >= 15; pos=pos-15) {  // go from 0-180 degrees
  //   servo.write(pin_servo_head, pos);        // set the servo position (degrees)
  //   delay(20);
  // }
  // for (int pos = 15; pos <= 90; pos=pos+15) {  // go from 0-180 degrees
  //   servo.write(pin_servo_head, pos);        // set the servo position (degrees)
  //   delay(20);
  // }
  // servo.write(pin_servo_head, 165);
  // delay(15);
  // servo.write(pin_servo_head, 15);
  // delay(15);
  // servo.write(pin_servo_head, 90);
  // delay(15);
}

// void moveArms() {
//   servo.write(pin_servo_rightarm, 165);
//   servo.write(pin_servo_leftarm, 15);
//   delay(100);

//   servo.write(pin_servo_rightarm, 15);
//   servo.write(pin_servo_leftarm, 165);
//   delay(100);

//   servo.write(pin_servo_rightarm, 90);
//   servo.write(pin_servo_leftarm, 90); 
//   delay(100);
// }

void switchCurrentScreen() {
  int currbuttonval_LEFT = digitalRead(pin_button_LEFT);
  int currbuttonval_RIGHT = digitalRead(pin_button_RIGHT);
  // Serial.println("left: " + String(currbuttonval_LEFT));
  // Serial.println("right: " + String(currbuttonval_RIGHT));
  // int currbuttonval_MENU = digitalRead(pin_button_MENU);
  
  switch(currentScreen) { 
    case Home:
      if (currbuttonval_LEFT == LOW && prevbuttonval_LEFT == HIGH) {
        currentScreen = Math;
        tft.fillScreen(ILI9341_BLACK);
        mathGame();
      }
      else if (currbuttonval_RIGHT == LOW && prevbuttonval_RIGHT == HIGH) {
        currentScreen = Trivia;
        tft.fillScreen(ILI9341_BLACK);
        triviaGame();
      }
    break;
    case Math:
      // if (currbuttonval_MENU == LOW && prevbuttonval_MENU == HIGH) {
      //   currentScreen = Home;
      //   tft.fillScreen(ILI9341_BLACK);
      //   tft.setTextColor(ILI9341_RED);
      //   tft.setCursor(tft.width()/6, tft.height()/3);
      //   tft.println("GAME OVER");
      //   delay(1000);
      //   if (curr_score > highscore_math) {
      //     highscore_math = curr_score;
      //   } 
      //   curr_score = 0;
      //   currentScreen = Home;
      //   tft.fillScreen(ILI9341_BLACK);
      //   homeScreen();
      // } 
      if (currbuttonval_LEFT == LOW && prevbuttonval_LEFT == HIGH && ans_leftorright == 0) {
        correctMelody();
        curr_score += 1;
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_GREEN);
        tft.setCursor(tft.width()/5, tft.height()/3);
        tft.println("CORRECT!");
        // delay(500);
        moveHead();
        tft.fillScreen(ILI9341_BLACK);
        mathGame();
      } else if (currbuttonval_LEFT == LOW && prevbuttonval_LEFT == HIGH && ans_leftorright == 1) {
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(tft.width()/6, tft.height()/3);
        tft.println("GAME OVER");
        incorrectMelody();
        // delay(500);
        // moveArms();
        moveHead();
        if (curr_score > highscore_math) {
          highscore_math = curr_score;
        }
        curr_score = 0;
        currentScreen = Home;
        tft.fillScreen(ILI9341_BLACK);
        homeScreen();
      } else if (currbuttonval_RIGHT == LOW && prevbuttonval_RIGHT == HIGH && ans_leftorright == 1) {
        correctMelody();
        curr_score += 1;
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_GREEN);
        tft.setCursor(tft.width()/5, tft.height()/3);
        tft.println("CORRECT!");
        // delay(500);
        moveHead();
        tft.fillScreen(ILI9341_BLACK);
        mathGame();
      } else if (currbuttonval_RIGHT == LOW && prevbuttonval_RIGHT == HIGH && ans_leftorright == 0) {
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(tft.width()/6, tft.height()/3);
        tft.println("GAME OVER");
        incorrectMelody();
        // delay(500);
        // moveArms();
        moveHead();
        if (curr_score > highscore_math) {
          highscore_math = curr_score;
        } 
        curr_score = 0;
        currentScreen = Home;
        tft.fillScreen(ILI9341_BLACK);
        homeScreen();
      }
    break; 
    case Trivia:
      // if (currbuttonval_MENU == LOW && prevbuttonval_MENU == HIGH) {
      //   currentScreen = Home;
      //   tft.fillScreen(ILI9341_BLACK);
      //   tft.setTextColor(ILI9341_RED);
      //   tft.setCursor(tft.width()/6, tft.height()/3);
      //   tft.println("GAME OVER");
      //   delay(1000);
      //   if (curr_score > highscore_trivia) {
      //     highscore_trivia = curr_score;
      //   } 
      //   curr_score = 0;
      //   currentScreen = Home;
      //   tft.fillScreen(ILI9341_BLACK);
      //   homeScreen();
      // } 
      if (currbuttonval_LEFT == LOW && prevbuttonval_LEFT == HIGH && ans_leftorright == 0) {
        correctMelody();
        curr_score += 1;
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_GREEN);
        tft.setCursor(tft.width()/5, tft.height()/3);
        tft.println("CORRECT!");
        // delay(500);
        moveHead();
        tft.fillScreen(ILI9341_BLACK);
        triviaGame();
      } else if (currbuttonval_LEFT == LOW && prevbuttonval_LEFT == HIGH && ans_leftorright == 1) {
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(tft.width()/6, tft.height()/3);
        tft.println("GAME OVER");
        incorrectMelody();
        // delay(500);
        //moveArms();
        moveHead();
        if (curr_score > highscore_trivia) {
          highscore_trivia = curr_score;
        }
        curr_score = 0;
        currentScreen = Home;
        tft.fillScreen(ILI9341_BLACK);
        homeScreen();
      } else if (currbuttonval_RIGHT == LOW && prevbuttonval_RIGHT == HIGH && ans_leftorright == 1) {
        correctMelody();
        curr_score += 1;
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_GREEN);
        tft.setCursor(tft.width()/5, tft.height()/3);
        tft.println("CORRECT!");
        // delay(500);
        moveHead();
        tft.fillScreen(ILI9341_BLACK);
        triviaGame();
      } else if (currbuttonval_RIGHT == LOW && prevbuttonval_RIGHT == HIGH && ans_leftorright == 0) {
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(tft.width()/6, tft.height()/3);
        tft.println("GAME OVER");
        incorrectMelody();
        // delay(500);
        // moveArms();
        moveHead();
        if (curr_score > highscore_trivia) {
          highscore_trivia = curr_score;
        } 
        curr_score = 0;
        currentScreen = Home;
        tft.fillScreen(ILI9341_BLACK);
        homeScreen();
      }

    break;

  }
  prevbuttonval_LEFT = currbuttonval_LEFT;
  prevbuttonval_RIGHT = currbuttonval_RIGHT;
  // prevbuttonval_MENU = currbuttonval_MENU;

}

void setup() {
  Serial.begin(9600);
  tft.begin();
  Serial.println(testFillScreen());
  pinMode(pin_button_LEFT, INPUT);
  pinMode(pin_button_RIGHT, INPUT);
  // pinMode(pin_button_MENU, INPUT_PULLUP);

  servo.write(pin_servo_head, 165);
  // servo.write(pin_servo_leftarm, 165);
  // servo.write(pin_servo_rightarm, 165);
  delay(500);

  servo.write(pin_servo_head, 90);
  // servo.write(pin_servo_leftarm, 90);
  // servo.write(pin_servo_rightarm, 90);
  delay(1000);
  // Serial.println(String(trivia_questions));

  Serial.print("Connecting to "); // prints that it is searching for connection
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { // prints status of wifi
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected"); // wifi status

  server.begin();
  Serial.println("Server started"); // server status

  Serial.print("AP IP Address: "); // once connected prints the IP address of the controller to which the code is uploaded
  Serial.println(WiFi.softAPIP());

  // sets all the motor pins as outputs
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  
}

void loop() {
  currMillis = millis();
  if (currMillis - prevMillis > 100) {
    switchCurrentScreen();
    prevMillis = currMillis;
  }

  // connect to server
  if (client.connect(serverIP, 80)) {
  
    // updates on connection, request from client
    client.println("ESP32 -- Client Side Connected"); 
    client.flush();

    // reading request from client
    String response = client.readStringUntil('\r');
    Serial.println("Response from server: " + response);
    
    // spacing responses val
    int firstSpaceIdx = response.indexOf(' ');
    int secondSpaceIdx = response.indexOf(' ', firstSpaceIdx+1);
    int thirdSpaceIdx = response.indexOf(' ', secondSpaceIdx +1);
    int fourthSpaceIdx = response.indexOf(' ', thirdSpaceIdx +1);
    int lastSpaceIdx = response.lastIndexOf(' ');
    // extracting the values between the spaces of printed responses
    String BlueButton = response.substring(firstSpaceIdx + 1, secondSpaceIdx);
    String RedButton = response.substring(response.indexOf("RedButton=") + 10, response.indexOf("GreenButton=") - 1);
    String GreenButton = response.substring(response.indexOf("GreenButton=") + 12, response.indexOf("YellowButton=") - 1);
    String YellowButton = response.substring(lastSpaceIdx-1);
    // converts to integer values  
    int blue = BlueButton.toInt();
    int red = RedButton.toInt();
    int green = GreenButton.toInt();
    int yellow = YellowButton.toInt();

    // printing out button values
    Serial.println(blue);
    Serial.println(red);
    Serial.println(green);
    Serial.println(yellow);

    // when no buttons are clicked the robot is not moving   
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);


    // controling DC motor 
    if (blue == 0) { // Blue button pressed, both motors move forwards
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
     }
    if (red == 0) { // Red button pressed, both motors move backwards
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
    }
    if (green == 0) { // Green button pressed, turning left
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
    }
    if (yellow == 0) { // Yellow button pressed, turning right
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
    }
  }
  else {
    Serial.println("Connection failed"); // prints if no connection
  }


}

