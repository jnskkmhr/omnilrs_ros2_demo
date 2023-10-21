#include <micro_ros_arduino.h>

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
std_msgs__msg__Float32MultiArray msg_float32arr;

#define LED_PIN 13

// left motor pin setup
#define CW_PIN_L 32
#define CCW_PIN_L 33
#define ANALOG_PIN_L 25

// right motor pin setup
#define CW_PIN_R 14
#define CCW_PIN_R 27
#define ANALOG_PIN_R 26

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  std_msgs__msg__Float32MultiArray * msg = (std_msgs__msg__Float32MultiArray *)msgin;

  float throttle_r = msg->data.data[0];
  float throttle_l = msg->data.data[1];

  int duty_l = 0;
  int duty_r = 0;

  // left motor
  if (throttle_l > 0){
    duty_l = 255 * throttle_l;
    digitalWrite(CW_PIN_L, HIGH);
    digitalWrite(CCW_PIN_L, LOW);
  }
  else if (throttle_l < 0){
    duty_l = -255 * throttle_l;
    digitalWrite(CW_PIN_L, LOW);
    digitalWrite(CCW_PIN_L, HIGH);
  }
  else if (throttle_l == 0){
    duty_l = 0;
    digitalWrite(CW_PIN_L, LOW);
    digitalWrite(CCW_PIN_L, LOW);
  }

  // right motor
  if (throttle_r > 0){
    duty_r = 255 * throttle_r;
    digitalWrite(CW_PIN_R, LOW);
    digitalWrite(CCW_PIN_R, HIGH);
  }
  else if (throttle_r < 0){
    duty_r = -255 * throttle_r;
    digitalWrite(CW_PIN_R, HIGH);
    digitalWrite(CCW_PIN_R, LOW);
  }
  else if (throttle_r == 0){
    duty_r = 0;
    digitalWrite(CW_PIN_R, LOW);
    digitalWrite(CCW_PIN_R, LOW);
  }

  dacWrite(ANALOG_PIN_L, duty_l);
  dacWrite(ANALOG_PIN_R, duty_r);
}

void setup() {
  // serial communication
  set_microros_transports();

  // error handling LED pin setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // motor driver pin setting
  pinMode(CW_PIN_L, OUTPUT);
  pinMode(CCW_PIN_L, OUTPUT);

  pinMode(CW_PIN_R, OUTPUT);
  pinMode(CCW_PIN_R, OUTPUT);

  // DAC setting
  pinMode(ANALOG_PIN_L, OUTPUT);
  pinMode(ANALOG_PIN_R, OUTPUT);

  // memory allocator
  allocator = rcl_get_default_allocator();

   //create support class
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node (&node, node_name, namespace, &support)
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "throttle"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_float32arr, &subscription_callback, ON_NEW_DATA));

  // allocate memory
  // msg_float32arr has two float32 array (so, 2*4=8byte)
  msg_float32arr.data.data = (float *)malloc(2 * sizeof(float));
  msg_float32arr.data.size = 0;
  msg_float32arr.data.capacity = 4;
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // delay(10);
}