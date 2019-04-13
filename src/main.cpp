// --------------------------------------
// Name: main.cpp
// Project: scara-mc-master
// Description: main function for the arduino master, relay ros messages to slaves
//---------------------------------------

#include <Arduino.h>
#include <ros.h>
#include <scara_master/AxisClicks.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <comm.h>

#define POSITION_TOPIC "GetPos"
#define CLICK_TOPIC "WheelEncoderClicks"
#define DRIVE_TO_TOPIC "DriveTo"
#define DRIVE_DIST_TOPIC "DriveDist"
#define HOME_TOPIC "Home"
#define PID_STATE_TOPIC "SetPidState"

ros::NodeHandle nh;
scara_master::AxisClicks click_msg;
scara_master::AxisClicks position_msg;
std_msgs::Int16 position;

// ================================= Variables ==============================
bool pid_state = false;

// ================================= ROS Functions =================================

// callback functions for Ros Subscribers
void HomeCb ( const std_msgs::Empty& toggle_msg ) {
    home();
}
void SetPidStateCb ( const std_msgs::Empty& toggle_msg ) {
  pid_state = !pid_state;
  set_pid_state(pid_state);
}
void DriveDistCb ( const scara_master::AxisClicks& clicks ) {
    int data[7];
    data[0] = clicks.zAxis;
    data[1] = clicks.Shoulder;
    data[2] = clicks.UAE;
    data[3] = clicks.UAJ;
    data[4] = clicks.SKE;
    data[5] = clicks.SKF;
    data[6] = clicks.SKG;
    drive_dist_max(data);
}
void DriveToCb ( const scara_master::AxisClicks& clicks ) {
    int data[7];
    data[0] = clicks.zAxis;
    data[1] = clicks.Shoulder;
    data[2] = clicks.UAE;
    data[3] = clicks.UAJ;
    data[4] = clicks.SKE;
    data[5] = clicks.SKF;
    data[6] = clicks.SKG;
    drive_to(data);
}

// define ROS publishers
ros::Publisher GetPosition(POSITION_TOPIC, &position_msg);
//ros::Publisher GetPosition(POSITION_TOPIC, &position);

// define ROS subscribers
ros::Subscriber<scara_master::AxisClicks> DriveTo(DRIVE_TO_TOPIC, &DriveToCb );
ros::Subscriber<scara_master::AxisClicks> DriveDistance(DRIVE_DIST_TOPIC, &DriveDistCb );
ros::Subscriber<std_msgs::Empty> Home(HOME_TOPIC, &HomeCb );
ros::Subscriber<std_msgs::Empty> SetPidState(PID_STATE_TOPIC, &SetPidStateCb );

// =================================================================================

void setup() {

    // Daniels test code starts here, must run before any ROS code! Comment out
    test();
    //Test function contains an infinite while-loop, if not commented out code will not prograss past this point!
    // Daniels test code ends here
    //Initialise Ros Node, publisher and subsribers
    Serial.begin(9600);    // initialize serial port
    nh.initNode();
    nh.advertise(GetPosition);
    nh.subscribe(DriveTo);
    nh.subscribe(DriveDistance);
    nh.subscribe(Home);
    nh.subscribe(SetPidState);

    //Set baud rate for Ros serial communication
    nh.getHardware()->setBaud(57600);

    //initialise communication to slaves (uses Serial1)
    init_Comm();
}

void loop() {
    // wait until the node handle has connected to ROS
    while(!nh.connected()) {
        nh.spinOnce();
    }

    int motor_count[7];
    // ACHTUNG! darf nicht durch andere befehle unterbrochen werden!
    // Wann werden die ROS CB Befehle ausgeführt? sind das wie Interrupts?
    get_position(motor_count);
    // publish clicks to Ros
    position_msg.zAxis = motor_count[0];
    position_msg.Shoulder = motor_count[1];
    position_msg.UAE = motor_count[2];
    position_msg.UAJ = motor_count[3];
    position_msg.SKE = motor_count[4];
    position_msg.SKF = motor_count[5];
    position_msg.SKG = motor_count[6];

    GetPosition.publish( &position_msg );

    //cyclical communication with Ros Master
    nh.spinOnce();
    _delay_ms(500);
}
