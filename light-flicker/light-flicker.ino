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
                              touch sensor includes
*******************************************************************************/
#include <CapacitiveSensor.h>
#include <MedianFilter.h>


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
                              touch sensor variables
*******************************************************************************/
MedianFilter test(20, 0);
CapacitiveSensor capSensor = CapacitiveSensor(4, 2);
int threshold = 200;
//const int ledPin = LED_BUILTIN;
bool led_state = LOW;
bool pass = false;
int lastLedState = HIGH;


void setup() {

  /******************************************************************************
                                touch sensor setup
   *******************************************************************************/
  //none
  
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
                                touch sensor/ radio code
  *******************************************************************************/
  long sensorValue = capSensor.capacitiveSensor(30);
  test.in( sensorValue);
  sensorValue = test.out();

  Serial.println(sensorValue);

  if (sensorValue > threshold) {
    if ( pass == false) {
      pass = true;
      led_state = !led_state;
      //digitalWrite(ledPin, led_state);
    }
  } else {
    pass = false;
  }
  
  if (led_state != lastLedState) {
    toggle();
  }





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

