// --------------------------------------
// Name: comm.cpp
// Project: scara-mc-master
// Description: implementation of multiserial communication
//---------------------------------------

#include <Arduino.h>

//--------------------
// VARIABLES
//--------------------
#define SLAVE_COUNT 2 //normally 7
#define BAUD_RATE 9600

const int ss_pin[7] = {22,23,24,25,26,27,28}; //Slave select lines tell slaves when they can use the bus
const int led_pin = 13; //Pin of onborad LED

//possible commands and respective command number (see for reference commands.md)
enum Command {
  c_ping = 0,
  c_home = 1,
  c_set_pid_state = 5,
  c_get_position = 6,
  c_drive_dist = 10,
  c_drive_dist_max = 11,
  c_drive_to = 12
};

//possible errors
enum Errortype {
  no_error = 0,
  e_wrong_slave = 1,
  e_ping_bad_echo = 2
};

//--------------------
// INTERNAL FUNCTIONS
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
int start_transmission(HardwareSerial &S1, int slave){
  serial_clear(S1);
  digitalWrite(ss_pin[slave],1); // enable slave to use bus
  int ready_msg = serial_read_int(S1); // wait for ready message
  if ( ready_msg != slave) {   // check validity of ready message
    digitalWrite(ss_pin[slave],0); // Error! disallow slave to use bus
    return e_wrong_slave;
  }
  return no_error;
}

//end command transmission by resetting ss pin
void end_transmission(int slave){
  digitalWrite(ss_pin[slave],0);
}

// // send a command to slaves (deprecated and untested)
// int send_command(HardwareSerial &S1, int command, int data[SLAVE_COUNT], int r_data[SLAVE_COUNT]){
//   //Command is sent to each Slave
//   int e = no_error
//   for (int slave = 0; slave < SLAVE_COUNT; slave++) {
//     e=start_transmission(S1,slave);   //Set slave select
//     serial_write_int(S1,command);   //Send command
//     serial_write_int(S1, data[slave]);   //Send data
//     r_data[slave] = serial_read_int(S1); //Receive data
//     end_transmission(slave);  //Release slave select
//   }
//   return e;
// }

//--------------------
// EXTERNAL FUNCTIONS
//--------------------

// Command: init_Comm
// Description: Initialise all pins and the Serial bus needed for the communication library
void init_Comm(){
  //initialise pins
  for (size_t i = 0; i < SLAVE_COUNT; i++) {
    pinMode(ss_pin[i], OUTPUT);
  }
  //set all pins to zero
  for (size_t i = 0; i < SLAVE_COUNT; i++) {
    digitalWrite(ss_pin[i], 0);
  }
  //start serial communication to slaves
  Serial1.begin(BAUD_RATE);
}

// Command: ping
// Description: Sends data to slaves, that each slave echoes back
int ping_slave(int ping, int echo[SLAVE_COUNT]){
  //error handling not implemented!
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_ping);   //Send command
    serial_write_int(Serial1, ping);   //Send data
    echo[slave] = serial_read_int(Serial1); //Receive data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

// Command: home
// Description: Set current position to zero
int home(){
  //error handling not implemented!
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_home);   //Send command
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

// Command:set_pid_state
// Description: Set PID of all slaves to ON or OFF
int set_pid_state(bool state){ //send command to each slave
  //error handling not implemented!
  int i_state = (int)state;
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_set_pid_state);   //Send command
    serial_write_int(Serial1, i_state);   //Send data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

// Command:get_position
// Description: request current position from slaves
int get_position(int motor_count[SLAVE_COUNT]){ //send command to each slave
  //error handling not implemented!
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_get_position);   //Send command
    motor_count[slave] = serial_read_int(Serial1); //Receive data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

// Command:drive_dist
// Description: Change target position by distance
int drive_dist(int distance[SLAVE_COUNT]){
  //error handling not implemented!
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_dist);   //Send command
    serial_write_int(Serial1, distance[slave]);   //Send data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

// Command: drive_dist_max
// Description: Move joint by distance from current position
int drive_dist_max(int distance[SLAVE_COUNT]){
  //error handling not implemented!
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_dist_max);   //Send command
    serial_write_int(Serial1, distance[slave]);   //Send data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

// Command: drive_to
// Description: Move joint to position
int drive_to(int position[SLAVE_COUNT]){
  //error handling not implemented!
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_to);   //Send command
    serial_write_int(Serial1, position[slave]);   //Send data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

//--------------------------
// Test Function
// from https://www.arduino.cc/en/Tutorial/ReadASCIIString
//--------------------------
void test(){//HardwareSerial &Serial, HardwareSerial &Serial1) {
  init_Comm();
  Serial.begin(9600);

  while (true) {
    Serial.println("Please enter the function you want to test using this format:");
    Serial.println("command_code, command_data");
    Serial.println("for testing the same data is sent to all slaves");

    // look for the next valid integer in the incoming serial stream:
    int command = Serial.parseInt();
    int data = Serial.parseInt();

    // look for the newline. That's the end of the input:
    while (Serial.read() != '\n') {
      delay(50);
    }

    Serial.println("Command: " + command);
    Serial.println("data: " + data);

    Serial.println("Press 'y' to send command, press any key to cancel");
    if (Serial.read() == 'y') {
      //generate data array
      int data_array[SLAVE_COUNT];
      // initializing array elements
      for (int i = 0; i < SLAVE_COUNT ; i++){
        data_array[i] = data;
      }
      int result_array[SLAVE_COUNT];
      int error_code=0;

      //select with command to run
      switch (command) {
        case 0: error_code=ping_slave(data, result_array);
        break;
        case 1: error_code=home();
        break;
        case 5: error_code=set_pid_state((bool) data);
        break;
        case 6: error_code=get_position(result_array);
        break;
        case 10: error_code=drive_dist(data_array);
        break;
        case 11: error_code=drive_dist_max(data_array);
        break;
        case 12: error_code=drive_to(data_array);
        break;
        default:
        error_code=9999;
      }
      if (error_code==0) {
        Serial.println("Command completed with no errors");
      } else {
        Serial.println("Command returned error: " + error_code);
      }

      Serial.print("Data Array: ");
      for (int i = 0; i < SLAVE_COUNT ; i++){
        Serial.print(data_array[i] + ", ");
      }
      Serial.println("");

      Serial.print("Result Array: ");
      for (int i = 0; i < SLAVE_COUNT ; i++){
        Serial.print(result_array[i] + ", ");
      }
      Serial.println("");
    }
    delay(50);
  }
}
