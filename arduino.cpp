#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
long maxIR = 0;
long minIR = 20000000;
float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop() {
  long irValue = particleSensor.getIR();
  long delta = millis() - lastBeat;
  if (Serial.available()){
    byte command = Serial.read();
    if (command == 0x41){
      lastBeat = millis();
      maxIR = 0;
      minIR = 2000000;
    }
  }
  if (irValue < 100000){
    maxIR = 0;
    minIR = 2000000;
    return;
  }
  if (delta > 20000){
    lastBeat = millis();
    maxIR = 0;
    minIR = 2000000;
  }
  if (irValue > maxIR){
    maxIR = irValue+1000; 
  }
  if (irValue < minIR){
    minIR = irValue-500;
  }
  Serial.print("MAX:");
  Serial.print(maxIR);
  Serial.print(",MIN:");
  Serial.print(minIR);
  Serial.print(",IR:");
  Serial.println(irValue);
}
