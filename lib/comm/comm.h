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

int ping_slave(int slave, int ping);
int home();
int set_pid_state(bool state);
int get_position(int motor_count[]);
int drive_dist(int distance[]);
int drive_dist_max(int distance[]);
int drive_to(int distance[]);

#endif
