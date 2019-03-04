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
const int ledPin = 3;



/******************************************************************************
                              servo variables
*******************************************************************************/
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position

/******************************************************************************
                              button variables
*******************************************************************************/
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;    // the number of the pushbutton pin

// Variables will change:
int led_state = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  /******************************************************************************
                                servo setup
  *******************************************************************************/
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  /******************************************************************************
                                radio setup
  *******************************************************************************/
  Serial.begin(9600); //open serial port
  pinMode(ledPin, OUTPUT);

  delay(50);
  radio.begin(); //start radio
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  digitalWrite(ledPin, led_state);


  /******************************************************************************
                                button setup
  *******************************************************************************/
  pinMode(buttonPin, INPUT);
}

void loop() {

  /******************************************************************************
                                radio code
  *******************************************************************************/
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    led_state = !led_state;
    digitalWrite(ledPin, led_state);
    Serial.println(text);
  }



  /******************************************************************************
                                button code
  *******************************************************************************/
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        led_state = !led_state;
      }
    }
   
  }
   // set the LED:
  digitalWrite(ledPin, led_state);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;

  


  /******************************************************************************
                                servo code
  *******************************************************************************/
  //for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //delay(15);                       // waits 15ms for the servo to reach the position
  //}
  //for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //  delay(15);                       // waits 15ms for the servo to reach the position
  //}

}
