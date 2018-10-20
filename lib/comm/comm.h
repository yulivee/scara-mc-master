#ifndef COMM_H_INCLUDED
#define COMM_H_INCLUDED

void test(HardwareSerial &S1, HardwareSerial &S0, int *result);

int ping_slave(HardwareSerial &Serial, int slave, int message, int *error_handler);
int drive_dist();
int drive_to();
int drive_dist_max();
int home();
int set_pid_state();
int get_position();

#endif
