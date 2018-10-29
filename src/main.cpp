#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <comm.h>

#define POSITION_TOPIC "GetPos"
#define CLICK_TOPIC "WheelEncoderClicks"
#define DRIVE_TO_TOPIC "DriveTo"
#define DRIVE_DIST_TOPIC "DriveDist"
#define HOME_TOPIC "Home"
#define PID_STATE_TOPIC "SetPidState"
#define DEBUG_TOPIC "Debug"

ros::NodeHandle nh;
std_msgs::Int16MultiArray click_msg;
std_msgs::Int16MultiArray position_msg;
std_msgs::String debug_msg;

int toggle_startup = 1;

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

// define ROS publishers
ros::Publisher GetPosition(POSITION_TOPIC, &position_msg);
ros::Publisher Debug(DEBUG_TOPIC, &debug_msg);

//void DriveDistCb ( const std_msgs::Int16MultiArray& clicks ) { drive_dist(clicks.data); }
void DriveToCb ( const std_msgs::Int16MultiArray& clicks ) { drive_to(clicks.data); position_msg.data = clicks.data; GetPosition.publish( &position_msg );}
void HomeCb ( const std_msgs::Empty& toggle_msg ) { home(); }
void SetPidStateCb ( const std_msgs::Empty& toggle_msg ) { set_pid_state(); }

// define ROS subscribers
ros::Subscriber<std_msgs::Int16MultiArray> DriveTo(DRIVE_TO_TOPIC, &DriveToCb );
//ros::Subscriber<std_msgs::Int16MultiArray> DriveDistance(DRIVE_DIST_TOPIC, &DriveDistCb );
ros::Subscriber<std_msgs::Empty> Home(HOME_TOPIC, &HomeCb );
ros::Subscriber<std_msgs::Empty> SetPidState(PID_STATE_TOPIC, &SetPidStateCb );

// =================================================================================

void setup() {

   char pos_label = "position";
   int data[7] = { 0, 0, 0, 0, 0, 0, 0 };
   position_msg.data_length = 7;
   position_msg.layout.dim[0].label = pos_label;
   position_msg.layout.dim[0].size = 7;
   position_msg.layout.dim[0].stride = 1*7;
   position_msg.layout.data_offset = 0;
   position_msg.data = data;

  // Daniels test code starts here, must run before any ROS code! Comment out
  init_Comm();
  //Test function contains an infinite while-loop, if not commented out code will not prograss past this point!
  // Daniels test code ends here

  //send hug to working buddy

  //Initialise Ros Node, publisher and subsribers
  nh.initNode();
  nh.advertise(GetPosition);
  nh.advertise(Debug);
  nh.subscribe(DriveTo);
  //nh.subscribe(DriveDistance);
  nh.subscribe(Home);
  nh.subscribe(SetPidState);

  //Set baud rate for Ros serial communication
  nh.getHardware()->setBaud(57600);
  nh.spinOnce();
}

void loop() {
  // wait until the node handle has connected to ROS
  while(!nh.connected()) {nh.spinOnce();}

  if ( toggle_startup == 1 ) {
          delay(5);
          debug_msg.data = "Scara-Bot Master ready for some serious mischief";
          Debug.publish(&debug_msg);
          toggle_startup = 0;
  }

  //publish clicks to Ros
  GetPosition.publish( &position_msg );

  //cyclical communication with Ros Master
  nh.spinOnce();
  _delay_ms(500);
}
