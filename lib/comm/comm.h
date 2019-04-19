// --------------------------------------
// Name: comm.h
// Project: scara-mc-master
// Description: header for comm.cpp
//---------------------------------------

#ifndef COMM_H_INCLUDED
#define COMM_H_INCLUDED

#include <Arduino.h>

void test();
void init_Comm();

void ping_slave(int ping,int echo[]);
void home();
void set_pid_state(bool state);
void get_position(int motor_count[]);
void drive_dist(int distance[]);
void drive_dist_max(int distance[]);
void drive_to(int distance[]);
void set_speed(int speed);
void set_zone(int zone);
void check_target_reached(bool target_reached[]);

extern byte slave_error;
extern byte master_error;


#endif
