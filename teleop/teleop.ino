#include <Arduino.h>
#include <Adafruit_MotorShield.h>

#include <Wire.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>

// Define PWM message structure
typedef struct
{
  int32_t pwbl;
  int32_t pwmr;
} pwm_msg_t;
rcl_publisher_t publisher;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t pwbl_subscription;
rcl_subscription_t pwmr_subscription;
std_msgs__msg__Int32 pwbl_msg;
std_msgs__msg__Int32 pwmr_msg;

// Existing motor control code
char key = NULL;
unsigned long currentMillis = 0;
long lastMotorCommand = 10000; // AUTO_STOP_INTERVAL
int motorSpeed = 50;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x40);

#define IFDAC_ENABLED 1

#define FRONTLEFT 0
#define FRONTRIGHT 1
#define BACKRIGHT 2
#define BACKLEFT 3

Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(4);

void initMotorController() {
  if (IFDAC_ENABLED) {
    // DAC configuration code (commented out for now)
    // ...
  }

  AFMS.begin(); // create with the default frequency 1.6KHz

  frontLeftMotor->setSpeed(100);
  backLeftMotor->setSpeed(100);
  frontRightMotor->setSpeed(100);
  backRightMotor->setSpeed(100);

  // set motors forward to initiate
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  frontRightMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  // release all motors
  frontLeftMotor->run(RELEASE);
  backLeftMotor->run(RELEASE);
  frontRightMotor->run(RELEASE);
  backRightMotor->run(RELEASE);
}

void setMotorSpeed(int motor, int spd) {
  switch (motor) {
    case FRONTLEFT:
      if (spd == 0)
        frontLeftMotor->run(RELEASE);
      else
        spd > 0 ? frontLeftMotor->run(FORWARD) : frontLeftMotor->run(BACKWARD);
      frontLeftMotor->setSpeed(abs(spd));
      break;
    case FRONTRIGHT:
      if (spd == 0)
        frontRightMotor->run(RELEASE);
      else
        spd > 0 ? frontRightMotor->run(FORWARD) : frontRightMotor->run(BACKWARD);
      frontRightMotor->setSpeed(abs(spd));
      break;
    case BACKRIGHT:
      if (spd == 0)
        backRightMotor->run(RELEASE);
      else
        spd > 0 ? backRightMotor->run(FORWARD) : backRightMotor->run(BACKWARD);
      backRightMotor->setSpeed(abs(spd));
      break;
    case BACKLEFT:
      if (spd == 0)
        backLeftMotor->run(RELEASE);
      else
        spd > 0 ? backLeftMotor->run(FORWARD) : backLeftMotor->run(BACKWARD);
      backLeftMotor->setSpeed(abs(spd));
      break;
  }
}

void setMotorSpeeds(int frontLeftSpeed, int frontRightSpeed, int backRightSpeed, int backLeftSpeed) {
  Serial.print("F1 ");
  Serial.print(frontLeftSpeed);
  Serial.print(" F2 ");
  Serial.println(frontRightSpeed);
  Serial.print("B1");
  Serial.print(backRightSpeed);
  Serial.print("B2 ");
  Serial.println(backLeftSpeed);
  setMotorSpeed(FRONTLEFT, frontLeftSpeed);
  setMotorSpeed(FRONTRIGHT, frontRightSpeed);
  setMotorSpeed(BACKRIGHT, -backRightSpeed);
  setMotorSpeed(BACKLEFT, -backLeftSpeed);
}

// Callback function for handling PWBL messages
void pwbl_callback(const void *msg_recv)
{
  const std_msgs__msg__Int32 *received_data = (const std_msgs__msg__Int32 *)msg_recv;
  int32_t pwbl = received_data->data;
  setMotorSpeeds(pwbl, 0, 0, pwbl); // Adjust motor speeds based on received PWM left value
}

// Callback function for handling PWMR messages
void pwmr_callback(const void *msg_recv)
{
  const std_msgs__msg__Int32 *received_data = (const std_msgs__msg__Int32 *)msg_recv;
  int32_t pwmr = received_data->data;
  setMotorSpeeds(0, pwmr, pwmr, 0); // Adjust motor speeds based on received PWM right value
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(33, 32); // sda= 33 /scl= 32 for sense and 22, 21 for ttgo_motor controller
  initMotorController();

  // Initialize micro-ROS
  // Set up support and allocator
  rclc_support_init(&support, 0, NULL, &allocator);

  // Set up executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  // Set up ROS node
  rclc_node_init_default(&node, "motor_control_node", "", &support);

  // Set up PWBL subscription
  rclc_subscription_init_default(
      &pwbl_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/pwbl_topic");

  // Add PWBL subscription to the executor
  rclc_executor_add_subscription(
      &executor,
      &pwbl_subscription,
      &pwbl_msg,
      &pwbl_callback,
      ON_NEW_DATA);

  // Set up PWMR subscription
  rclc_subscription_init_default(
      &pwmr_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/pwmr_topic");

  // Add PWMR subscription to the executor
  rclc_executor_add_subscription(
      &executor,
      &pwmr_subscription,
      &pwmr_msg,
      &pwmr_callback,
      ON_NEW_DATA);
}

void loop()
{
  // Spin the executor to handle incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  currentMillis = millis();
  if (currentMillis > lastMotorCommand) { // Check to see if we have exceeded the auto-stop interval
    setMotorSpeeds(0, 0, 0, 0);
  }
}
