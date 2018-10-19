#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#define NODE "Skg"
#define WEC "WheelEncoderClicks"
#define DT "DriveTo"
#define DD "DriveDist"
#define HOME "Home"
#define TOG "ToggleMotor"

#define CLICK_TOPIC WEC NODE
#define DRIVE_TO_TOPIC DT NODE
#define DRIVE_DIST_TOPIC DD NODE
#define HOME_TOPIC HOME NODE
#define TOGGLE_TOPIC TOG NODE

ros::NodeHandle nh;
std_msgs::Int32 click_msg;
std_msgs::Int32 position_msg;

// ================================= ROS Functions =================================

// callback functions for Ros Subscribers
// we need this per axis
void drive_dist_cb ( const std_msgs::Int32& clicks ) {
        target_position += clicks.data;
}

// we need this per axis
void drive_to_cb ( const std_msgs::Int32& clicks ) {
        target_position = clicks.data;
}

// we need this per axis
void home_cb ( const std_msgs::Empty& toggle_msg ) {
        motor_cnt = 0;
        target_position = motor_cnt;
}

// we need this per axis
void toggle_motor_cb ( const std_msgs::Empty& toggle_msg ) {
    target_position = motor_cnt;
    digitalWrite(motor_pins.left, 0);
    digitalWrite(motor_pins.right, 0);
    digitalWrite(motor_pins.enable, !digitalRead(motor_pins.enable));
}

// define ROS publishers
ros::Publisher wheel_encoder_clicks(CLICK_TOPIC, &click_msg);
ros::Publisher get_position(POSITION_TOPIC, &position_msg);

// define ROS subscribers
ros::Subscriber<std_msgs::Int32> drive_to(DRIVE_TO_TOPIC, &drive_to_cb );
ros::Subscriber<std_msgs::Int32> drive_distance(DRIVE_DIST_TOPIC, &drive_dist_cb );
ros::Subscriber<std_msgs::Empty> home(HOME_TOPIC, &home_cb );
ros::Subscriber<std_msgs::Empty> toggle_motor(TOGGLE_TOPIC, &toggle_motor_cb );

// =================================================================================

void serial_write_int(int val){
  Serial1.write(lowByte(val));
  Serial1.write(highByte(val));
}

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);
  
    //Initialise Ros Node, publisher and subsribers
    nh.initNode();
    nh.advertise(get_position);
    nh.subscribe(drive_to);
    nh.subscribe(drive_distance);
    nh.subscribe(home);
    nh.subscribe(toggle_motor);
    
    //Set baud rate for Ros serial communication
    nh.getHardware()->setBaud(57600);
}

void loop() {
  // wait until the node handle has connected to ROS
  while(!nh.connected()) {nh.spinOnce();}

  // read from port 1(Slave), send to port 0(PC):
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // read from port 0 (PC), send to port 1(Slave):
  if (Serial.available()) {
    int inByte = Serial.read();
    serial_write_int(-300);
    //Serial1.write(inByte);
  }
    //publish clicks to Ros
    click_msg.data = motor_cnt;
    wheel_encoder_clicks.publish( &click_msg );
    test.publish( &test_msg );

    //cyclical communication with Ros Master
    nh.spinOnce();
    _delay_ms(500);
}
