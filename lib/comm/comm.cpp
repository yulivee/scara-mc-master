#include <Arduino.h>

//--------------------
// VARIABLES
//--------------------
const int slave_count = 7;
//priming wires tell slaves when they can use the bus
const int prime_pins[7] = {22,23,24,25,26,27,28};
// fire wire to run a command on all slaves at once
const int fire_pin = 29;
//led pin for debugging
const int led_pin = 13;

//--------------------
// INTERNAL FUNCTIONS
//--------------------

// read integer as 2 byte package from serial bus 1
int serial_read_int(HardwareSerial &S1){
  byte byte_buffer[2];
  S1.readBytes(byte_buffer,2); //Store the next 2 Bytes of serial data in the buffer
  // convert buffer to conv_integer
  int val = ((byte_buffer[1]) << 8) + byte_buffer[0];
  return val;
}

// write integer as 2 byte package to serial bus 1
void serial_write_int(HardwareSerial &S1, int val){
  S1.write(lowByte(val));
  S1.write(highByte(val));
}

//clear serial input buffer
void serial_clear(HardwareSerial &S1){
  while (S1.available() > 0) {
    S1.read();
  }
}

// send a command and command data to target slave, error returns 1, success returns 0
int send_command(int slave, int command, int data){
  serial_clear(Serial1);
  // prime slave
  digitalWrite(prime_pins[slave],1);
  // wait for ready message (slave sends its number)
  int ready_msg = serial_read_int(Serial1);
  // check validity of ready message
  if ( ready_msg != slave) {
    // Error!
    return 1;
  }
  //send command
  serial_write_int(Serial1,command);
  //send data
  serial_write_int(Serial1,data);
  digitalWrite(slave,0);
  return 0;
}

// primes all slaves, sending each a command and command data
void run_command(HardwareSerial &Serial, int command, int data[slave_count], int l_slave_count){
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

//--------------------
// EXTERNAL FUNCTIONS
//--------------------

//Initialise all pins needed for the communication library
void init_Comm(){
  for (size_t i = 0; i < slave_count; i++) {
      pinMode(prime_pins[i], OUTPUT);
  }
  pinMode(fire_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin,1);
  delay(100);
  digitalWrite(led_pin,0);
}

//Command: Ping, Command Number: 0
//sends a command data package to a single slave that the slave echoes back, error returns -1, success returns 1
int ping_slave(int slave, int message){
  if (send_command(slave,0,message) < 0) {
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

void test(){//HardwareSerial &Serial, HardwareSerial &Serial1) {
  while (true) {

     if (Serial.available()) {
       digitalWrite(led_pin,1);
       int inByte = Serial.read();
       Serial.println(inByte);
         //ping slave;
       Serial.println("Running Ping");
        //ping slave 0
         Serial.println("Pinging slave 0:");
         int result =  ping_slave(0, 11111);
         Serial.println(result);
        //ping slave 1
         Serial.println("Pinging slave 1:");
         result =  ping_slave(2, 22222);
         Serial.println(result);
    }

    delay(50);
    digitalWrite(led_pin,0);
  }
}

int drive_dist( int clicks ){
  return 0;
}
int drive_to( int clicks ){
  return 0;
};
int drive_dist_max(){
  return 0;
};
int home(){
  return 0;
};
int set_pid_state(){
  return 0;
};
int get_node_positions(){
  return 0;
};
