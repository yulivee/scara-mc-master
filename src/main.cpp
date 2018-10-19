#include <Arduino.h>
#include <comm.h>

//--------------------
// PINOUT
// 0: RX0 - reserved for USB communication to ROS
// 1: TX0 - reserved USB communication to ROS
// 18: TX1 (mislabled) - communication bus to slaves
// 19: RX1 (mislabeld) - communication bus to slaves
//--------------------
int error_handler = 0;

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  // read from port 1(Slave), send to port 0(PC):
  // if (Serial1.available()) {
  //   int inByte = Serial1.read();
  //   Serial.write(inByte);
  // }

  // read from port 0 (PC), send to port 1(Slaves):
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial.println(inByte);
      //ping slave;
      test(&Serial1, &Serial);
      Serial.println("Running Ping");
      //ping slave 0
      Serial.println("Pinging slave 0:");
      Serial.println(ping_slave(0, 11111,&error_handler));


      //ping slave 1
      Serial.println("Pinging slave 1:");
      Serial.println(ping_slave(1, 22222,&error_handler));

  }
}
