#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Empty.h>
#include <comm.h>

#define POSITION_TOPIC "GetPos"
#define CLICK_TOPIC "WheelEncoderClicks"
#define DRIVE_TO_TOPIC "DriveTo"
#define DRIVE_DIST_TOPIC "DriveDist"
#define HOME_TOPIC "Home"
#define PID_STATE_TOPIC "SetPidState"

ros::NodeHandle nh;
std_msgs::Int16MultiArray click_msg;
std_msgs::Int16MultiArray position_msg;

//--------------------
// PINOUT
// 0: RX0 - reserved for USB communication to ROS
// 1: TX0 - reserved USB communication to ROS
// 18: TX1 (mislabled) - communication bus to slaves
// 19: RX1 (mislabeld) - communication bus to slaves
//--------------------

// ================================= ROS Functions =================================

// callback functions for Ros Subscribers
//                                 Int16MultiArray
void DriveDistCb ( const std_msgs::Int16MultiArray& clicks ) { drive_dist(clicks.data); }
void DriveToCb ( const std_msgs::Int16MultiArray& clicks ) { drive_to(clicks.data); }
void HomeCb ( const std_msgs::Empty& toggle_msg ) { home(); }
void SetPidStateCb ( const std_msgs::Empty& toggle_msg ) { set_pid_state(); }

// define ROS publishers
ros::Publisher GetPosition(POSITION_TOPIC, &position_msg);

// define ROS subscribers
ros::Subscriber<std_msgs::Int16MultiArray> DriveTo(DRIVE_TO_TOPIC, &DriveToCb );
ros::Subscriber<std_msgs::Int16MultiArray> DriveDistance(DRIVE_DIST_TOPIC, &DriveDistCb );
ros::Subscriber<std_msgs::Empty> Home(HOME_TOPIC, &HomeCb );
ros::Subscriber<std_msgs::Empty> SetPidState(PID_STATE_TOPIC, &SetPidStateCb );

// =================================================================================

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);

  // Daniels test code starts here, must run before any ROS code! Comment out
  init_Comm();
  // test();
  //Test function contains an infinite while-loop, if not commented out code will not prograss past this point!
  // Daniels test code ends here

    //Initialise Ros Node, publisher and subsribers
    nh.initNode();
    nh.advertise(GetPosition);
    nh.subscribe(DriveTo);
    nh.subscribe(DriveDistance);
    nh.subscribe(Home);
    nh.subscribe(SetPidState);

    //Set baud rate for Ros serial communication
    nh.getHardware()->setBaud(57600);
}

void loop() {
  // wait until the node handle has connected to ROS
  while(!nh.connected()) {nh.spinOnce();}

    //publish clicks to Ros
    click_msg.data = get_node_positions();
    GetPosition.publish( &click_msg );

    //cyclical communication with Ros Master
    nh.spinOnce();
    _delay_ms(500);
}
