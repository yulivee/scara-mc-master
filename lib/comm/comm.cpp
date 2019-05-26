// --------------------------------------
// Name: comm.cpp
// Project: scara-mc-master
// Description: implementation of multiserial communication
//---------------------------------------

#include <Arduino.h>
#include <data_def.h>

//--------------------
// VARIABLES AND CONSTANTS
//--------------------
#define BAUD_RATE 9600 //Boud Rate for communicationg with slaves

const int ss_pin[7] = {22,23,24,25,26,27,28}; //Slave select lines tell slaves when they can use the bus
const int led_pin = 13; //Pin of onborad LED

//error handling
byte slave_error[SLAVE_COUNT]; //Errors returned by slaves
byte master_error[SLAVE_COUNT]; //errors that ocur on the master

//--------------------
// SPI PROTOCOL FUNCTIONS
//--------------------

// read integer as 2 byte package from serial bus
int serial_read_int(HardwareSerial &S1){
  byte byte_buffer[2];
  S1.readBytes(byte_buffer,2); //Store the next 2 Bytes of serial data in the buffer
  // convert buffer to conv_integer
  int val = ((byte_buffer[1]) << 8) + byte_buffer[0];
  return val;
}

// write integer as 2 byte package to serial bus
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

//start a command transmission on serial bus by setting ss pin and doing handshake
void start_transmission(HardwareSerial &S1, int slave){
  digitalWrite(led_pin,true);
  serial_clear(S1);
  digitalWrite(ss_pin[slave],true); // enable slave to use bus
  int ready_msg = serial_read_int(S1); // wait for ready message
  if ( ready_msg != slave+1) {   // check validity of ready message
    digitalWrite(ss_pin[slave],false); // Error! disallow slave to use bus
    digitalWrite(led_pin,false);
    master_error[slave]=e_wrong_slave;
  }
}

//end command transmission by resetting ss pin
void end_transmission(HardwareSerial &S1, int slave, int command){
  digitalWrite(ss_pin[slave],false);   //reset bus
  int error_code = serial_read_int(S1); //Receive slave error data
  if (error_code != command) {  //check if eror ocurred
    slave_error[slave]=error_code;
  }
  digitalWrite(led_pin,false);
}

//--------------------
// EXTERNAL FUNCTIONS
//--------------------

// Command: init_Comm
// Description: Initialise all pins and the Serial bus needed for the communication library
void init_Comm(){
  //initialise pins
  for (size_t i = 0; i < 7; i++) {
    pinMode(ss_pin[i], OUTPUT);
    digitalWrite(ss_pin[i], 0);
  }
  //start serial communication to slaves
  Serial1.begin(BAUD_RATE);
  Serial1.setTimeout(50);
  //turn off motors
  //set_pid_state(false);
}

// Command: ping
// Description: Sends data to slaves, that each slave echoes back
void ping_slave(int ping, int echo[SLAVE_COUNT]){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_ping);   //Send command
    serial_write_int(Serial1, ping);   //Send data
    echo[slave] = serial_read_int(Serial1); //Receive data
    end_transmission(Serial1, slave, c_ping);  //Release slave select
  }
}

// Command: home
// Description: Set current position to zero
void home(){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_home);   //Send command
    end_transmission(Serial1, slave,c_home);  //Release slave select
  }
}

// Command:set_pid_state
// Description: Set PID of all slaves to ON or OFF
void set_pid_state(bool state){ //send command to each slave
  int i_state = (int)state;
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_set_pid_state);   //Send command
    serial_write_int(Serial1, i_state);   //Send data
    end_transmission(Serial1, slave,c_set_pid_state);  //Release slave select
  }
}

// Command:get_position
// Description: request current position from slaves
void get_position(int motor_count[SLAVE_COUNT]){ //send command to each slave
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_get_position);   //Send command
    motor_count[slave] = serial_read_int(Serial1); //Receive data
    end_transmission(Serial1, slave,c_get_position);  //Release slave select
  }
}

// Command:get_target
// Description: request current target from slaves (debug function)
void get_target(int target[SLAVE_COUNT]){ //send command to each slave
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_get_target);   //Send command
    target[slave] = serial_read_int(Serial1); //Receive data
    end_transmission(Serial1, slave,c_get_target);  //Release slave select
  }
}

// Command:c_get_slave_num
// Description:
void get_slave_num(int slave_numbers[SLAVE_COUNT]){ //send command to each slave
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_get_slave_num);   //Send command
    slave_numbers[slave] = serial_read_int(Serial1); //Receive data
    end_transmission(Serial1, slave,c_get_slave_num);  //Release slave select
  }
}

// Command:drive_dist
// Description: Change target position by distance
void drive_dist(int distance[SLAVE_COUNT]){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_dist);   //Send command
    serial_write_int(Serial1, distance[slave]);   //Send data
    end_transmission(Serial1, slave,c_drive_dist);  //Release slave select
  }
}

// Command: drive_dist_max
// Description: Move joint by distance from current position
void drive_dist_max(int distance[SLAVE_COUNT]){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_dist_max);   //Send command
    serial_write_int(Serial1, distance[slave]);   //Send data
    end_transmission(Serial1, slave,c_drive_dist_max);  //Release slave select
  }
}

// Command: drive_to
// Description: Move joint to position
void drive_to(int position[SLAVE_COUNT]){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_to);   //Send command
    serial_write_int(Serial1, position[slave]);   //Send data
    end_transmission(Serial1, slave,c_drive_to);  //Release slave select
  }
}

//Command: set_speed
// Description: define speed of the robot in percent
void set_speed(int speed){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_set_speed);   //Send command
    serial_write_int(Serial1, speed);   //Send data
    end_transmission(Serial1, slave,c_set_speed);  //Release slave select
  }
}

// Command: set_zone
// Description: define fly-by distance to target when it counts as "reached"
void set_zone(int zone){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_set_zone);   //Send command
    serial_write_int(Serial1, zone);   //Send data
    end_transmission(Serial1, slave,c_set_zone);  //Release slave select
  }
}

// Command: check_target_reached
// Description: Check if Slaves have reached target position (target within zone)
bool check_target_reached(){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_check_target_reached);   //Send command
    bool target_reached = (bool) serial_read_int(Serial1); //Receive data
    end_transmission(Serial1, slave,c_check_target_reached);  //Release slave select
    if (target_reached==false) {
      return false;
    }
  }
  return true;
}
