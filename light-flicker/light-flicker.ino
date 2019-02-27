/******************************************************************************
                              servo includes
*******************************************************************************/
#include <Servo.h>

/******************************************************************************
                              radio includes
*******************************************************************************/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/******************************************************************************
                              button includes
*******************************************************************************/
//none

/******************************************************************************
                              radio variables
*******************************************************************************/
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

/******************************************************************************
                              servo variables
*******************************************************************************/
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position

/******************************************************************************
                              button variables
*******************************************************************************/
int inPin = 2;         // the number of the input pin
int outPin = 13;       // the number of the output pin

int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

void setup() {
  /******************************************************************************
                                servo setup
  *******************************************************************************/
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  /******************************************************************************
                                radio setup
  *******************************************************************************/
  Serial.begin(9600); //open serial port
  radio.begin(); //start radio
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  /******************************************************************************
                                button setup
  *******************************************************************************/
  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);
}

void loop() {

  /******************************************************************************
                                radio code
  *******************************************************************************/
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }



  /******************************************************************************
                                button code
  *******************************************************************************/
  reading = digitalRead(inPin);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == HIGH)
      state = LOW;
    else
      state = HIGH;

    time = millis();
  }

  digitalWrite(outPin, state);

  previous = reading;

  /******************************************************************************
                                servo code
  *******************************************************************************/
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
