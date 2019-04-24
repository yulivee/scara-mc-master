// --------------------------------------
// Name: data_def.h
// Project: scara-mc-master
// Description: definition of global constants
//---------------------------------------

#define SLAVE_COUNT 7 //Slaves connected to the master, normally 7

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

//possible Error returns
enum Errortype {
  no_error = 0,
  e_wrong_slave = 91,
  e_ping_bad_echo = 92,
  e_unknown_command = 93,
  e_bad_data = 94,
  default_value = 99
};
