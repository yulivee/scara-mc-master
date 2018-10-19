#ifndef COMM_H_INCLUDED
#define COMM_H_INCLUDED

#ifndef __cplusplus
extern "C" {
#endif

void test();

int ping_slave();
int drive_dist();
int drive_to();
int drive_dist_max();
int home();
int set_pid_state();
int get_position();


#ifndef __cplusplus
}
#endif

#endif
