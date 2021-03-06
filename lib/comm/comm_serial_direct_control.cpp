// --------------------------------------
// Name: comm_serial_direct_cotrol.cpp
// Project: scara-mc-master
// Description: implementation of functions to controll master via serial and not via ROS
// -------------------------------------

#include <Arduino.h>
#include <comm.h>
#include <data_def.h>

//--------------------------
// Direct Communication Main Funion
// expects a command of up to 8 characters:
// command_number, data0, data1, data2 ... etc.
//Commands that only expect a simgle input variable (eg. set_pid_state) use the first data element of the received array
//
// To use this library just put this at the start auf de setup code:
// //Daniels test code starts here, must run before any ROS code! Comment out
// direct_comm_main(); //Test function contains an infinite while-loop, if not commented out code will not prograss past this point!
// //Daniels test code ends here
//--------------------------

void direct_comm_main(){
  //SETUP
  init_Comm(); //Initialize slave communication
  Serial.begin(9600); //start Communication with pc

  //LOOP
  while(true){
    //Local variables for command reception
    String inString = "";
    int inArray[SLAVE_COUNT+1];
    for (int i = 0; i < SLAVE_COUNT+1; i++) {
      inArray[i]=1;
    }

    serial_clear(Serial);
    //receive command
    int j=0;
    while(j < (SLAVE_COUNT+1)) {
      while(Serial.available() == 0){
        delay(50);
      }
      int inChar = Serial.read();     // Read serial input:
      //parse received character
      if (isDigit(inChar)) {
        // convert the incoming byte to a char and add it to the string:
        inString += (char)inChar;
      }else if (inChar==',') {  //commas seperate the elements in the array
        inArray[j]=inArray[j]*inString.toInt();
        j++;
        inString = "";
      }else if (inChar =='-') { //is the data naegative?
        inArray[j]=-1;
      }else if (inChar == '\n') { //end of input
        inArray[j]=inArray[j]*inString.toInt();
        j=SLAVE_COUNT+1;  //End of input!
        inString = "";
      }
    }

    //extract command and data from received array
    int command = inArray[0];
    int data_array[SLAVE_COUNT];
    for (int i = 0; i < SLAVE_COUNT ; i++){
      data_array[i] = inArray[i+1];
    }

    //DEBUG echo the entrered Command and Data
    // Serial.print("\n");
    // Serial.print("Command: ");
    // Serial.print(command);
    //
    // Serial.print(" Data Array: [");
    // for (int i = 0; i < SLAVE_COUNT ; i++){
    //   Serial.print(data_array[i]);
    //   Serial.print(" ");
    // }
    // Serial.print("]\n");
    //DEBUG end

    //initialise result arrays
    int result_array[SLAVE_COUNT];  //int array
    for (int i = 0; i < SLAVE_COUNT ; i++){
      result_array[i] = default_value;
    }
    //bool bool_array[SLAVE_COUNT]; //bool array
    //for (int i = 0; i < SLAVE_COUNT ; i++){
    //  bool_array[i] = false;
    //}
    bool target_reached = false;
    int waitTime = 0;
    int waitTimeMax = 500; //set to 500 for offline mode, set higher for online mode

    bool expect_return = false;
    //select with command to run
    switch (command) {
      case c_ping:
      ping_slave(data_array[0],result_array);
      expect_return=true;
      break;
      case c_home:
      home();
      break;
      case c_set_pid_state:
      set_pid_state((bool) data_array[0]);
      break;
      case c_get_position:
      get_position(result_array);
      expect_return=true;
      break;
      case c_get_target:
      get_target(result_array);
      expect_return=true;
      break;
      case c_get_slave_num:
      get_slave_num(result_array);
      expect_return=true;
      break;
      case c_drive_dist:
      drive_dist(data_array);
      break;
      case c_drive_dist_max:
      drive_dist_max(data_array);
      break;
      case c_drive_to:
        drive_to(data_array);
        // while ((!check_target_reached()) && (waitTime<waitTimeMax)) {
        // _delay_ms(100);
        //   waitTime = waitTime + 100;
        // }
        // if (true){//waitTime<waitTimeMax) { //Set to always true for oflline
        //   Serial.print("target_reached\n");
        // }else{
        //   Serial.print("movement_timeout!\n");
        // }
      break;
      case c_set_speed:
      set_speed(data_array[0]);
      break;
      case c_set_zone:
      set_zone(data_array[0]);
      break;
      case c_check_target_reached:
      target_reached = check_target_reached();
      expect_return=true;
      break;
      default:
      break;
    }

    //send master error data
    Serial.print("m");
    for (int i = 0; i < SLAVE_COUNT ; i++){
      Serial.print(",");
      Serial.print(master_error[i]);
    }
    Serial.print(" ");

    //send slave error data
    Serial.print("s");
    for (int i = 0; i < SLAVE_COUNT ; i++){
      Serial.print(",");
      Serial.print(slave_error[i]);
    }
    Serial.print("\n");

    //if necesarry send return data
    if (expect_return) {
      if (command == c_check_target_reached) {
          Serial.print(target_reached);
        }else{
        for (int i = 0; i < SLAVE_COUNT-1 ; i++){
          Serial.print(result_array[i]);
          Serial.print(",");
        }
        Serial.print(result_array[SLAVE_COUNT-1]);
      }
      Serial.print("\n");
    }
  }

}
