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

// ================================= ROS Functions =================================

// callback functions for Ros Subscribers
void DriveDistCb ( const scara_master::AxisClicks& clicks ) {
    int data[7];
    data[0] = clicks.zAxis;
    data[1] = clicks.Shoulder;
    data[2] = clicks.UAE;
    data[3] = clicks.UAJ;
    data[4] = clicks.SKE;
    data[5] = clicks.SKF;
    data[6] = clicks.SKG;
    drive_dist(data);
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
void HomeCb ( const std_msgs::Empty& toggle_msg ) {
    home();
}
void SetPidStateCb ( const std_msgs::Empty& toggle_msg ) {
    set_pid_state();
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
    // initialize both serial ports:
    Serial.begin(9600);
    Serial1.begin(9600);

    // Daniels test code starts here, must run before any ROS code! Comment out
    init_Comm();
    //test();
    //Test function contains an infinite while-loop, if not commented out code will not prograss past this point!
    // Daniels test code ends here

    //send hug to working buddy

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
    while(!nh.connected()) {
        nh.spinOnce();
    }


    //publish clicks to Ros
    position_msg.zAxis = 1;
    position_msg.Shoulder = 2;
    position_msg.UAE = 3;
    position_msg.UAJ = 4;
    position_msg.SKE = 5;
    position_msg.SKF = 6;
    position_msg.SKG = 7;

// click_msg.data = get_node_positions();
    GetPosition.publish( &position_msg );
    //position.data = 42;
    //GetPosition.publish( &position );

    //cyclical communication with Ros Master
    nh.spinOnce();
    _delay_ms(500);
}
