#include <Arduino.h>

//--------------------
// PINOUT
// 0: RX0 - reserved for USB communication to ROS
// 1: TX0 - reserved USB communication to ROS
// 18: TX1 (mislabled) - communication bus to slaves
// 19: RX1 (mislabeld) - communication bus to slaves
//--------------------
const int slave_count = 7;
const int prime_pins[slave_count] = {22,23,24,25,26,27,28}; //priming wires tell slaves when they can use the bus
const int fire_pin = 29; // fire wire to run a command on all slaves at once

//--------------------
// VARIABLES
//--------------------

//--------------------
// FUNCTIONS
//--------------------

// write integer as 2 byte package to serial bus 1
void serial1_write_int(int val){
  Serial1.write(lowByte(val));
  Serial1.write(highByte(val));
}

// read integer as 2 byte package from serial bus 1
int serial1_read_int(){
  byte byte_buffer[2];
  Serial1.readBytes(byte_buffer,2); //Store the next 2 Bytes of serial data in the buffer
  // convert buffer to conv_integer
  int val = ((byte_buffer[1]) << 8) + byte_buffer[0];
  return val;
}

// send a command and command data to target slave, returns -1 if an error occurs
int send_command(int slave, int command, int data){
  // prime slave
  digitalWrite(slave,1);
  // wait for ready message (slave, sends its number)
  int ready_msg = serial1_read_int();
  // check validity of ready message
  if ( ready_msg != slave) {
    // Error
    return -1;
  }
  //send command
  serial1_write_int(command);
  //send data
  serial1_write_int(data);
  digitalWrite(slave,0);
  return 1;
}

// primes all slaves, sending each send a command and command data
void run_command(int command, int data[slave_count]){
  for (int slave = 0; slave < slave_count; slave++) {
    //send command to the slave, with appropriate data atached
    send_command(slave, command, data[slave]);
    // TODO: Error handling not implemented!
  }
  digitalWrite(fire_pin,1);
  delayMicroseconds(50);
  digitalWrite(fire_pin,0);
}

//--------------------
// MAIN
//--------------------
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
    //int inByte = Serial.read();
    serial1_write_int(-300);
    //Serial1.write(inByte);
  }
}
