#include <Arduino.h>

void serial_write_int(int val){
  Serial1.write(lowByte(val));
  Serial1.write(highByte(val));
}

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  // read from port 1(Slave), send to port 0(PC):
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // read from port 0 (PC), send to port 1(Slave):
  if (Serial.available()) {
    int inByte = Serial.read();
    serial_write_int(-300);
    //Serial1.write(inByte);
  }
}
