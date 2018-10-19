#include <Arduino.h>

//--------------------
// PINOUT
// 0: RX0 - reserved for USB communication to ROS
// 1: TX0 - reserved USB communication to ROS
// 18: TX1 (mislabled) - communication bus to slaves
// 19: RX1 (mislabeld) - communication bus to slaves
//--------------------

//Error handler umsetzten als Globale variable, überlegung ob nur 1 verschachtelung

const int slave_count = 7;
const int prime_pins[7] = {22,23,24,25,26,27,28}; //priming wires tell slaves when they can use the bus
const int fire_pin = 29; // fire wire to run a command on all slaves at once
int null_data[7] = {0,0,0,0,0,0,0};
int error_handler = 0;

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

//
void serial1_clear(){
  while (Serial1.available() > 0) {
    Serial1.read();
  }
}

// send a command and command data to target slave, error returns -1, success returns 1
int send_command(int slave, int command, int data){
  serial1_clear();
  // prime slave
  digitalWrite(prime_pins[slave],1);
  // wait for ready message (slave sends its number)
  int ready_msg = serial1_read_int();
  // check validity of ready message
  if ( ready_msg != slave) {
    // Error!
    return -1;
  }
  //send command
  serial1_write_int(command);
  //send data
  serial1_write_int(data);
  digitalWrite(slave,0);
  return 1;
}

// primes all slaves, sending each a command and command data
void run_command(int command, int data[slave_count], int l_slave_count){
  for (int l_slave = 0; l_slave < l_slave_count; l_slave++) {
    //send command to the slave, with appropriate data attached
    if (send_command(l_slave, command, data[l_slave]) <0){
      //Error!
    }
  }
  digitalWrite(fire_pin,1);
  delayMicroseconds(50);
  digitalWrite(fire_pin,0);
}


//Command: Ping, Command Number: 0
//sends a command data package to a single slave that the slave echoes back, error returns -1, success returns 1
int ping_slave(int slave, int message, int *error_handler){
  if (send_command(slave,0,message) < 0) {
    //Error!
    //*error_handler= -1;
    //return;
  }

  int echo = serial1_read_int();
  if (echo != message) {
    //Error!
    //*error_handler= -2;
    return -1;
}
return 1;
}

void test(int *result) {
  *result = 1;
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
  // if (Serial1.available()) {
  //   int inByte = Serial1.read();
  //   Serial.write(inByte);
  // }

  // read from port 0 (PC), send to port 1(Slaves):
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial.println(inByte);
      //ping slave;
      Serial.println("Running Ping");
      //ping slave 0
      Serial.println("Pinging slave 0:");
      Serial.println(ping_slave(0, 11111,&error_handler));


      //ping slave 1
      Serial.println("Pinging slave 1:");
      Serial.println(ping_slave(1, 22222,&error_handler));

  }
}
