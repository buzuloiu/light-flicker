#include <CapacitiveSensor.h>
#include <MedianFilter.h>

MedianFilter test(20, 0);


CapacitiveSensor capSensor = CapacitiveSensor(4, 2);

int threshold = 800;
const int ledPin = LED_BUILTIN;
bool led_state = LOW;
bool pass = false;
void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  delay(50);
}

void loop()
{
  long sensorValue = capSensor.capacitiveSensor(30);
  test.in( sensorValue);
  sensorValue = test.out();
  
  Serial.println(sensorValue);
  
  if (sensorValue > threshold)
  {
     if( pass == false){
     pass = true; 
     led_state = !led_state;
    digitalWrite(ledPin, led_state);
     }
  }else{
      pass = false;
  }
  
  
}
