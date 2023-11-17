#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/logging_rosout.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

//Define the number of steps per revolution for your stepper motor.
const int stepsPerRevolution = 200;

//Define the pins for your stepper motor driver.
const int XstepPin = 2;
const int XdirPin = 5;
const int YstepPin = 3;
const int YdirPin = 6;
const int ZstepPin = 4;
const int ZdirPin = 7;
const int Enable = 8;


AccelStepper stepperX(1, XstepPin, XdirPin);  //1 = driver interface type
AccelStepper stepperY(1, YstepPin, YdirPin);  //1 = driver interface type
AccelStepper stepperZ(1, ZstepPin, ZdirPin);  //1 = driver interface type

MultiStepper steppersControl;

long gotoposition[3];

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

//rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{
	const  geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
	gotoposition[0] = msg->linear.x;
  gotoposition[1] = msg->linear.y;
  gotoposition[2] = msg->linear.z;

}
// ------------------------------------------------ROS related part------------------------------------------------
void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/humblebot/cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  msg.linear.x = 0;

  // ------------------------------------------------Stepper related part------------------------------------------------
  pinMode( Enable,OUTPUT);
  digitalWrite(Enable, LOW);
  
  //Set the maximum speed and acceleration in steps per second
  stepperX.setMaxSpeed(1000);
  stepperY.setMaxSpeed(1000);
  stepperZ.setMaxSpeed(1000);

  stepperX.setAcceleration(500);
  stepperY.setAcceleration(500);
  stepperZ.setAcceleration(500);

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  steppersControl.addStepper(stepperX);
  steppersControl.addStepper(stepperY);
  steppersControl.addStepper(stepperZ);
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  steppersControl.moveTo(gotoposition);
  steppersControl.runSpeedToPosition();

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);
}