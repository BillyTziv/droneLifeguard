#include <Servo.h> 
#define THROTTLE_PIN  11

Servo grabber;
unsigned long thrDuration;

const byte thrInPin = THROTTLE_PIN;

int pos =0;
int stickVal =0;

void setup() {
  pinMode(thrInPin, INPUT);
  grabber.attach(10);

  Serial.begin(9600);
}

int getGrabStickValue() {
    thrDuration = pulseIn(thrInPin, HIGH);
   
    
    Serial.print("Throttle: \t");
    Serial.print(thrDuration);
    Serial.print("\n");
    
    return thrDuration;
}

void loop() {
  stickVal = getGrabStickValue();
  if( (stickVal > 1700) && ((stickVal < 1900)) ) {
    // Open the grabber
    grabber.write(150);
   
    //Serial.print("Op: \t");
  }else {
    // Close the grabber
    grabber.write(30);
   
    //Serial.print("Throttle: \t");
  }  
  delay(50);
}
