// --------------------------------------
// Name: comm.cpp
// Project: scara-mc-master
// Description: implementation of multiserial communication
//---------------------------------------

#include <Arduino.h>

//--------------------
// VARIABLES
//--------------------
#define SLAVE_COUNT 2
//normally 7
#define BAUD_RATE 9600

//Slave select lines tell slaves when they can use the bus
const int ss_pin[7] = {22,23,24,25,26,27,28};
//Pin of onborad LED
const int led_pin = 13;
//possible commands and respective comman number (see for reference commands.md)
enum Command {
  c_ping = 0,
  c_home = 1,
  c_set_pid_state = 5,
  c_get_position = 6,
  c_drive_dist = 10,
  c_drive_dist_max = 11,
  c_drive_to = 12
};

//possible errors and respective numbers
enum Errortype {
  no_error = 0,
  e_wrong_slave,
  e_ping_bad_echo
};

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

int start_transmission(HardwareSerial &S1, int slave){
  serial_clear(S1);
  digitalWrite(ss_pin[slave],1); // prime slave
  int ready_msg = serial_read_int(S1); // wait for ready message
  if ( ready_msg != slave) {   // check validity of ready message
    // Error!
    digitalWrite(ss_pin[slave],0);
    return e_wrong_slave;
  }
  return no_error;
}

void end_transmission(int slave){
  digitalWrite(ss_pin[slave],0);
}

// send a command to slaves
int send_command(HardwareSerial &S1, int command, int data[SLAVE_COUNT], int r_data[SLAVE_COUNT]){
  //Command is sent to each Slave
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(S1,slave);   //Set slave select
    serial_write_int(S1,command);   //Send command
    serial_write_int(S1, data[slave]);   //Send data
    r_data[slave] = serial_read_int(S1); //Receive data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}



//--------------------
// EXTERNAL FUNCTIONS
//--------------------

//Initialise all pins needed for the communication library
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

//----------------------------------------------------------------------
// EXTERNAL FUNCTIONS
//----------------------------------------------------------------------

// Command: ping
// Description: Sends data to a single slave, that the slave echoes data back
int ping_slave(int slave, int ping){
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_ping);   //Send command
    serial_write_int(Serial1, ping);   //Send data
    int echo = serial_read_int(Serial1); //Receive data
    end_transmission(slave);  //Release slave select
    return echo;
  }

// Command: home
// Description: Set current position to zero
int home(){
    //Command is sent to each Slave
    for (int slave = 0; slave < SLAVE_COUNT; slave++) {
      start_transmission(Serial1,slave);   //Set slave select
      serial_write_int(Serial1,c_home);   //Send command
      end_transmission(slave);  //Release slave select
    }
    return no_error;
}

// Command:set_pid_state
// Description: Set PID of all slaves to ON or OFF
int set_pid_state(bool state){
    //Command is sent to each Slave
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
int get_position(int motor_count[SLAVE_COUNT]){
    //Command is sent to each Slave
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
    //Command is sent to each Slave
    for (int slave = 0; slave < SLAVE_COUNT; slave++) {
      start_transmission(Serial1,slave);   //Set slave select
      serial_write_int(Serial1,c_drive_dist);   //Send command
      serial_write_int(Serial1, distance[slave]);   //Send data
      end_transmission(slave);  //Release slave select
    }
return no_error;
}

// Command:
// Description:
int drive_dist_max(int distance[SLAVE_COUNT]){
  //Command is sent to each Slave
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_dist_max);   //Send command
    serial_write_int(Serial1, distance[slave]);   //Send data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

// Command:
// Description:
int drive_to(int position[SLAVE_COUNT]){
  //Command is sent to each Slave
  for (int slave = 0; slave < SLAVE_COUNT; slave++) {
    start_transmission(Serial1,slave);   //Set slave select
    serial_write_int(Serial1,c_drive_to);   //Send command
    serial_write_int(Serial1, position[slave]);   //Send data
    end_transmission(slave);  //Release slave select
  }
  return no_error;
}

//--------------------------
// Test Function
//--------------------------
void test(){//HardwareSerial &Serial, HardwareSerial &Serial1) {
  int result;
  while (true) {

    // if (Serial.available()) {
    //   digitalWrite(led_pin,1);
    //   char inByte = Serial.read();
    //   switch (inByte) {
    //     case 'p':
    //     //test ping
    //     Serial.println(inByte);
    //     //ping slave;
    //     Serial.println("Running Ping");
    //     //ping slave 0
    //     Serial.println("Pinging slave 0:");
    //     result =  ping_slave(0, 11111);
    //     Serial.println(result);
    //     //ping slave 1
    //     Serial.println("Pinging slave 1:");
    //     result =  ping_slave(1, 22222);
    //     Serial.println(result);
    //     case 'd':
    //     //test drive_to
    //     Serial.println("Running drive_dist");
    //     //int clicks[2] = {100,100};
    //     //result = drive_dist(clicks);
    //     //Serial.println(result);
    //   }
    //   serial_clear(Serial);
    //   digitalWrite(led_pin,0);
    // }
    //HardwareSerial &S1, int command, int data[SLAVE_COUNT], int r_data[SLAVE_COUNT]){
      //Command is sent to each Slave
      // for (int slave = 0; slave < SLAVE_COUNT; slave++) {
      //   start_transmission(Serial1,slave);   //Set slave select
      //   serial_write_int(Serial1,command);   //Send command
      //   serial_write_int(Serial1, data[slave]);   //Send data
      //   r_data[slave] = serial_read_int(Serial1); //Receive data
      //   end_transmission(slave);  //Release slave select
      // }

    delay(50);

  }
}
