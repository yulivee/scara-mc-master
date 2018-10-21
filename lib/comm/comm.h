#ifndef COMM_H_INCLUDED
#define COMM_H_INCLUDED

#include <Arduino.h>

void test();
void init_Comm();

int ping_slave(int slave, int message);
int drive_dist( int clicks );
int drive_to( int clicks );
int drive_dist_max();
int home();
int set_pid_state();
int get_node_positions();

#endif
