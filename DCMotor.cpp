#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <SIKTEC_SRAM.h>
#include <SdFat.h>
#include <Adafruit_GFX.h>

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

void setup() {
  Serial.begin(9600);// starts serial (for printing etc.)
  delay(10);

  // connecting to the Wi-Fi network, giving consistent updates
  Serial.println();
  Serial.println();
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
