#include <Arduino.h>

//--------------------
// VARIABLES
//--------------------
const int slave_count = 7;
const int prime_pins[7] = {22,23,24,25,26,27,28}; //priming wires tell slaves when they can use the bus
const int fire_pin = 29; // fire wire to run a command on all slaves at once
//Error handler umsetzten als Globale variable, überlegung ob nur 1 verschachtelung

//--------------------
// FUNCTIONS
//--------------------

// read integer as 2 byte package from serial bus 1
int serial_read_int(HardwareSerial &S1){
  byte byte_buffer[2];
  S1.readBytes(byte_buffer,2); //Store the next 2 Bytes of serial data in the buffer
  // convert buffer to conv_integer
  int val = ((byte_buffer[1]) << 8) + byte_buffer[0];
  return val;
}

//
void serial_clear(HardwareSerial &S1){
  while (S1.available() > 0) {
    S1.read();
  }
}

// write integer as 2 byte package to serial bus 1
void serial_write_int(HardwareSerial &Serial1,int val){
  Serial1.write(lowByte(val));
  Serial1.write(highByte(val));
}

// send a command and command data to target slave, error returns -1, success returns 1
int send_command(HardwareSerial &S1, int slave, int command, int data){
  serial_clear(S1);
  // prime slave
  digitalWrite(prime_pins[slave],1);
  // wait for ready message (slave sends its number)
  int ready_msg = serial_read_int(S1);
  // check validity of ready message
  if ( ready_msg != slave) {
    // Error!
    return -1;
  }
  //send command
  serial_write_int(S1,command);
  //send data
  serial_write_int(S1,data);
  digitalWrite(slave,0);
  return 1;
}




// primes all slaves, sending each a command and command data
void run_command(HardwareSerial &Serial, int command, int data[slave_count], int l_slave_count){
  for (int l_slave = 0; l_slave < l_slave_count; l_slave++) {
    //send command to the slave, with appropriate data attached
    if (send_command(Serial,l_slave, command, data[l_slave]) <0){
      //Error!
    }
  }
  digitalWrite(fire_pin,1);
  delayMicroseconds(50);
  digitalWrite(fire_pin,0);
}

//Command: Ping, Command Number: 0
//sends a command data package to a single slave that the slave echoes back, error returns -1, success returns 1
int ping_slave(HardwareSerial &Serial, int slave, int message, int *error_handler){
  if (send_command(Serial, slave,0,message) < 0) {
    //Error!
    //*error_handler= -1;
    //return;
  }

  int echo = serial_read_int(Serial);
  if (echo != message) {
    //Error!
    //*error_handler= -2;
    return -1;
}
return 1;
}

void test(HardwareSerial &S1, HardwareSerial &S0, int *result) {
  // send_command(S1, 0, 0, 0);
  // *result = 1;
  // if (Serial.available()) {
  //   int inByte = Serial.read();
  //   Serial.println(inByte);
  //     //ping slave;
  //     test(&Serial1, &Serial);
  //     Serial.println("Running Ping");
  //     //ping slave 0
  //     Serial.println("Pinging slave 0:");
  //     Serial.println(ping_slave(0, 11111,&error_handler));
  // 
  //
  //     //ping slave 1
  //     Serial.println("Pinging slave 1:");
  //     Serial.println(ping_slave(1, 22222,&error_handler));
}
