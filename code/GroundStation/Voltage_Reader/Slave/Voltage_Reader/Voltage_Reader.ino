#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
const int analogInPin = A0;  // Analog input pin that the measures the voltage
int sensorValue = 0;        // value read from the pot
double voltage;


void setup() {
pinMode(13, OUTPUT);
Serial.begin(9600); // start serial for output
// initialize i2c as slave
Wire.begin(SLAVE_ADDRESS);

// define callbacks for i2c communication
Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Ready!");
}

void loop() {
delay(100);
sensorValue = analogRead(analogInPin);
voltage = (3.3 * sensorValue / 1024) * ((1.5 + 1)/1);

  Serial.print("sensor Voltage = " );
  Serial.println(voltage);
}

// callback for received data
void receiveData(int byteCount){

while(Wire.available()) {
number = Wire.read();
Serial.print("data received: ");
Serial.println(number);

if (number == 1){

if (state == 0){
digitalWrite(13, HIGH); // set the LED on
state = 1;
}
else{
digitalWrite(13, LOW); // set the LED off
state = 0;
}
}
}
}

// callback for sending data
void sendData(){
  sensorValue = analogRead(analogInPin);
 //voltage = (3.3 * sensorValue / 1024) * ((1.5 + 1)/1);
Wire.write(sensorValue);
}
