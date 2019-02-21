/******************************************************************************
                              touch sensor includes
*******************************************************************************/
#include <CapacitiveSensor.h>
#include <MedianFilter.h>

/******************************************************************************
                              radio includes
*******************************************************************************/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/******************************************************************************
                              radio variables
*******************************************************************************/
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";


/******************************************************************************
                              touch sensor variables
*******************************************************************************/
MedianFilter test(20, 0);
CapacitiveSensor capSensor = CapacitiveSensor(4, 2);
int threshold = 800;
const int ledPin = LED_BUILTIN;
bool led_state = LOW;
bool pass = false;


void setup()
{
  /******************************************************************************
                                radio setup
  *******************************************************************************/
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  /******************************************************************************
                                touch sensor setup
  *******************************************************************************/
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  delay(50);
}

void loop()
{
  /******************************************************************************
                                touch sensor code
  *******************************************************************************/
  long sensorValue = capSensor.capacitiveSensor(30);
  test.in( sensorValue);
  sensorValue = test.out();

  Serial.println(sensorValue);

  if (sensorValue > threshold){
     if( pass == false){
     pass = true;
     led_state = !led_state;
     digitalWrite(ledPin, led_state);
    }
  }else{
    pass = false;
  }

  /******************************************************************************
                                radio code
  *******************************************************************************/
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));
  delay(1000);

}
