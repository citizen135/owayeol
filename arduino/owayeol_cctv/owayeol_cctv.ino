#include <Arduino.h>
#include <DFRobot_TFmini.h>
#include <Servo.h> 

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
//ROS 노드 클래스 변수
ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range("range_data", &range_msg);
SoftwareSerial mySerial(8, 7);
DFRobot_TFmini  TFmini;
float distance;

//서보 클래스 변수
Servo robot_servos[2];
char servo_pins[2] = {6, 5}; // PWM Pins on Arduino Uno
int mid_positions[2] = {90, 90};
int SERVO_CURRENT_POSITIONS[2];
char frameid[] = "/link3";
float TARGET_JOINT_POSITIONS[2] = {0,0};

void writeServos() {
  for (int j = 0; j < 2; j++) {
    int target_angle;
    if (j == 1) {
      // Due to difference in mounting directions
      target_angle = - TARGET_JOINT_POSITIONS[j]*(180/3.14) + mid_positions[j];
    } else {
      target_angle = TARGET_JOINT_POSITIONS[j]*(180/3.14) + mid_positions[j];
    }
    robot_servos[j].write(target_angle);
    SERVO_CURRENT_POSITIONS[j] = target_angle;
  }
  if (TFmini.measure()) {                  // 거리와 신호의 강도를 측정합니다. 성공하면 을 반환하여 if문이 작동합니다.
      distance = TFmini.getDistance();       // 거리값을 cm단위로 불러옵니다.
      range_msg.range=distance/10;
      range_msg.header.stamp = nh.now();
      pub_range.publish(&range_msg);
  }
  nh.spinOnce();
}

void servoControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  TARGET_JOINT_POSITIONS[0] = msg.position[0];
  TARGET_JOINT_POSITIONS[1] = msg.position[1];
  // Call the method to write the joint positions to the servo motors
  writeServos();

}
ros::Subscriber<sensor_msgs::JointState> servo_control_subscriber_joint_state("joint_states", &servoControlSubscriberCallbackJointState);

void setup() {
  // Initial the servo motor connections and initialize them at home position
  for (unsigned int i = 0; i < 2; i++) {
    robot_servos[i].attach(servo_pins[i]);
    robot_servos[i].write(mid_positions[i]);
    SERVO_CURRENT_POSITIONS[i] = mid_positions[i];
  }

  // Set the communication BaudRate and start the node
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(servo_control_subscriber_joint_state);
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.1;
  range_msg.max_range = 12;

  TFmini.begin(mySerial);
}

void loop() {
  // Keep calling the spinOnce() method in this infinite loop to stay tightly coupled with the ROS Serial
  nh.spinOnce();
  if (TFmini.measure()) {                  // 거리와 신호의 강도를 측정합니다. 성공하면 을 반환하여 if문이 작동합니다.
      distance = TFmini.getDistance();       // 거리값을 cm단위로 불러옵니다.
      range_msg.range=distance/10;
      range_msg.header.stamp = nh.now();
      pub_range.publish(&range_msg);
  }
  delay(5);
}
