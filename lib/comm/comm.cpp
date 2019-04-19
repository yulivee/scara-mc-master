// --------------------------------------
// Name: comm.cpp
// Project: scara-mc-master
// Description: implementation of multiserial communication
//---------------------------------------

#include <Arduino.h>

//--------------------
// VARIABLES
//--------------------
#define SLAVE_COUNT 1 //normally 7
#define BAUD_RATE 9600

const int ss_pin[7] = {22,23,24,25,26,27,28}; //Slave select lines tell slaves when they can use the bus
const int led_pin = 13; //Pin of onborad LED

//possible commands and respective command number (see for reference commands.md)
enum Command {
  c_ping = 0,
  c_home = 1,
  c_set_pid_state = 2,
  c_get_position = 6,
  c_get_target = 7,
  c_get_slave_num =8,
  c_drive_dist = 10,
  c_drive_dist_max = 11,
  c_drive_to = 12,
  c_set_speed = 15,
  c_set_zone = 16,
  c_check_target_reached = 20
};

//error handling
byte slave_error[SLAVE_COUNT];
byte master_error[SLAVE_COUNT];
enum Errortype {
  no_error = 0,
  e_wrong_slave = 91,
  e_ping_bad_echo = 92,
  e_unknown_command = 93,
  e_bad_data = 94,
  default_value = 99
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
void start_transmission(HardwareSerial &S1, int slave){
  digitalWrite(led_pin,true);
  serial_clear(S1);
  digitalWrite(ss_pin[slave],true); // enable slave to use bus
  int ready_msg = serial_read_int(S1); // wait for ready message
  //DEBUG:   Serial.println(ready_msg);
  if ( ready_msg != slave+1) {   // check validity of ready message
    digitalWrite(ss_pin[slave],false); // Error! disallow slave to use bus
    digitalWrite(led_pin,false);
    master_error[slave]=e_wrong_slave;
  }
}

//end command transmission by resetting ss pin
void end_transmission(HardwareSerial &S1, int slave, int command){
  //reset bus
  digitalWrite(ss_pin[slave],false);
  int error_code = serial_read_int(S1); //Receive data
  if (error_code != command) {
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
void check_target_reached(bool target_reached[SLAVE_COUNT]){
  for (int slave = 0; slave < SLAVE_COUNT; slave++) { //send command to each slave
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_check_target_reached);   //Send command
    target_reached[slave] = (bool) serial_read_int(Serial1); //Receive data
    end_transmission(Serial1, slave,c_check_target_reached);  //Release slave select
  }
}

//==============================================
//TEST Functions
//==============================================


//--------------------------
// Test Function
// from https://www.arduino.cc/en/Tutorial/ReadASCIIString
//--------------------------
void test(){//HardwareSerial &Serial, HardwareSerial &Serial1) {
  //SETUP
  init_Comm();
  Serial.begin(9600);

  //LOOP
  while(true){

  //set running mode (single slave or all)
  Serial.println("Please enter the slave number you wish to sent data to or enter 'a' for all (use 999,0 to reset)");
  while (Serial.available() == 0) { // wait for user input
    delay(50);
  }

  char s_input = Serial.read();
  int akt_slave = default_value;
  if ('0'<s_input && s_input<'8') {
    akt_slave=s_input-49;
    Serial.print("Slave ");
    Serial.print(akt_slave+1);
    Serial.println(" selected");
  }

  //Test Loop
  bool b=true;
  while (b) {
    serial_clear(Serial1);
    serial_clear(Serial);
    Serial.print("Please enter the function using this format: ");
    Serial.println("command, data, move dir (opt., default +)");

    while (Serial.available() == 0) {
      delay(50);
    }
    // look for the next valid integer in the incoming serial stream:
    int command = Serial.parseInt();
    int data = Serial.parseInt();

    // look for the newline. That's the end of the input:
    char c=Serial.read();
    while (c != '\n') {
      if (c == '-') {
        data = -data;
      }
      c=Serial.read();
      delay(50);
    }

    //echo the entrered Command and Data
    Serial.print("Command: ");
    Serial.print(command);
    Serial.print(" | Data: ");
    Serial.println(data);

    //generate data array
    int data_array[SLAVE_COUNT];
    for (int i = 0; i < SLAVE_COUNT ; i++){
      data_array[i] = data;
    }

    //if running single slave mode
    if (0<=akt_slave && akt_slave<7) {
      for (int i = 0; i < SLAVE_COUNT ; i++){
        data_array[i] = 0;
      }
      data_array[akt_slave]=data;
    }
    Serial.print("Data Array: [");
    for (int i = 0; i < SLAVE_COUNT ; i++){
      Serial.print(data_array[i]);
      Serial.print(" ");
    }
    Serial.println("]");

    //initialise result arrays
    int result_array[SLAVE_COUNT];
    for (int i = 0; i < SLAVE_COUNT ; i++){
      result_array[i] = default_value;
    }
    bool bool_array[SLAVE_COUNT];
    for (int i = 0; i < SLAVE_COUNT ; i++){
      bool_array[i] = false;
    }

    //select with command to run
    switch (command) {
      case c_ping:
      Serial.println("ping_slave");
      ping_slave(data,result_array);
      break;
      case c_home:
      Serial.println("home");
      home();
      break;
      case c_set_pid_state:
      Serial.println("set_pid_state");
      set_pid_state((bool) data);
      break;
      case c_get_position:
      Serial.println("get_position");
      get_position(result_array);
      break;
      case c_get_target:
      Serial.println("get_target");
      get_target(result_array);
      break;
      case c_get_slave_num:
      Serial.println("get_slave_num");
      get_slave_num(result_array);
      break;
      case c_drive_dist:
      Serial.println("drive_dist");
      drive_dist(data_array);
      break;
      case c_drive_dist_max:
      Serial.println("drive_dist_max");
      drive_dist_max(data_array);
      break;
      case c_drive_to:
      Serial.println("drive_to");
      drive_to(data_array);
      break;
      case c_set_speed:
      Serial.println("set_speed");
      set_speed(data);
      break;
      case c_set_zone:
      Serial.println("set_zone");
      set_zone(data);
      break;
      case c_check_target_reached:
      Serial.println("check_target_reached");
      check_target_reached(bool_array);
      break;
      case 999:
      b=false;
      break;
      default:
      Serial.println("Error: Unknown Command");
    }

    Serial.print("Errorhandler: M[");
      for (int i = 0; i < SLAVE_COUNT ; i++){
        Serial.print(master_error[i]);
        Serial.print(" ");
      }
      Serial.print("], S[");
      for (int i = 0; i < SLAVE_COUNT ; i++){
        Serial.print(slave_error[i]);
        Serial.print(" ");
      }
      Serial.println("]");

    Serial.print("Result Array: [");
    if (command == c_check_target_reached) {
      for (int i = 0; i < SLAVE_COUNT ; i++){
        Serial.print(bool_array[i]);
        Serial.print(" ");
      }
    }else{
      for (int i = 0; i < SLAVE_COUNT ; i++){
        Serial.print(result_array[i]);
        Serial.print(" ");
      }
    }
    Serial.println("]");
    Serial.println(" ");
    delay(50);
  }
}
}
