/******************************************************************************
                              general includes
*******************************************************************************/
#include <EEPROM.h>


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
                              general variables
*******************************************************************************/
int eAddress = 0; //EEPROM address


/******************************************************************************
                              radio variables
*******************************************************************************/
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001"; //radio address
const int ledPin = 3; //digital pin of the LED


/******************************************************************************
                              servo variables
*******************************************************************************/
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
const int servoUpperLimit = 140; //upper bound of pos
const int servoLowerLimit = 30; //lower bound of pos
const int servoPin = 6; //digital pin that servo is attached to


/******************************************************************************
                              button variables
*******************************************************************************/
const int buttonPin = 2;    // the number of the pushbutton pin

// Variables will change:
int led_state = HIGH;         // the current state of the LED pin
int lastLedState = HIGH;      //the last known state of the LED pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


void setup() {
  /******************************************************************************
                                servo setup
  *******************************************************************************/
  switchUp(); //initialize the switch in the down position


  /******************************************************************************
                                radio setup
  *******************************************************************************/
  Serial.begin(9600); //open serial port
  pinMode(ledPin, OUTPUT); //set LED as output

  delay(50);
  radio.begin(); //start radio
  radio.openReadingPipe(0, address); //open radui reading pipe
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  digitalWrite(ledPin, led_state); //set LED to initial state


  /******************************************************************************
                                button setup
  *******************************************************************************/
  pinMode(buttonPin, INPUT); //set button as an input


}

void loop() {

  /******************************************************************************
                                radio code
  *******************************************************************************/
  if (radio.available()) { //if the radio receives anything, toggle the LED state
    char text[32] = "";
    radio.read(&text, sizeof(text));
    led_state = !led_state;
    //Serial.println(text);
    if (led_state != lastLedState) {
      toggle();
    }

  }



  /******************************************************************************
                                button code
  *******************************************************************************/
  int reading = digitalRead(buttonPin); //read the button

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    lastDebounceTime = millis();     // reset the debouncing timer

  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    if (reading != buttonState) { // if the button state has changed:
      buttonState = reading;

      if (buttonState == HIGH) { // only toggle the LED if the new button state is HIGH
        led_state = !led_state;

      }
    }

  }
  if (led_state != lastLedState) {
    toggle();
  }



  lastButtonState = reading;   // save the reading. Next time through the loop, it'll be the lastButtonState:


  /******************************************************************************
                                  LED code
  *******************************************************************************/



  /******************************************************************************
                                 LED/servo code
   *******************************************************************************/
  if (led_state != lastLedState) {
    toggle();
  }
}


void toggle() {

  digitalWrite(ledPin, led_state);     // set the LED:

  if (lastLedState) { //move the switch down if the last LED state was ON
    switchDown();
    //EEPROM.update(eAddress, led_state); //save servo state to eeprom
  }
  else { //move the switch up if the last LED state was OFF
    switchUp();
    //EEPROM.update(eAddress, led_state);
  }
  Serial.print("LED: ");
  Serial.println(led_state);
  Serial.print("Last LED: ");
  Serial.println(lastLedState);
  lastLedState = !lastLedState;
}


void switchUp() {
  
    if (!myservo.attached()) {
    myservo.attach(servoPin);  // attaches the servo on pin servoPin to the servo object
    }

    for (pos ; pos <= servoUpperLimit; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15ms for the servo to reach the position
    }


    for (pos ; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);
    }

    myservo.detach();  // attaches the servo on pin 4 to the servo object
  
}


void switchDown() {
  
    if (!myservo.attached()) {
    myservo.attach(servoPin);  // attaches the servo on pin  servoPin to the servo object
    }

    for (pos ; pos >= servoLowerLimit; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15ms for the servo to reach the position
    }
    


    for (pos ; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15ms for the servo to reach the position
    }
    myservo.detach();  // attaches the servo on pin 4 to the servo object
  
}

