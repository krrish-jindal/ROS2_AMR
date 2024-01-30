#include <Arduino.h>
#include <Adafruit_MotorShield.h>

#include <Wire.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/string.h>  


// Define PWM message structure
typedef struct
{
  int32_t pwml;
  int32_t pwmr;
} pwm_msg_t;
rcl_publisher_t publisher;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t pwml_subscription;
rcl_subscription_t pwmr_subscription;
std_msgs__msg__Int32 pwml_msg;
std_msgs__msg__Int32 pwmr_msg;

rcl_publisher_t motor_speeds_publisher;
std_msgs__msg__String motor_speeds_msg;


// Existing motor control code
char key ;
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
  
  // Set motor speeds
  setMotorSpeed(FRONTLEFT, frontLeftSpeed);
  setMotorSpeed(FRONTRIGHT, frontRightSpeed);
  setMotorSpeed(BACKRIGHT, -backRightSpeed);
  setMotorSpeed(BACKLEFT, -backLeftSpeed);

  // Publish motor speeds
  char motor_speeds_str[50];  // Adjust the buffer size based on your needs
  snprintf(motor_speeds_str, sizeof(motor_speeds_str), "%d,%d,%d,%d", frontLeftSpeed, frontRightSpeed, backRightSpeed, backLeftSpeed);
  
  // Assign the C string to the std_msgs__msg__String data field
  motor_speeds_msg.data.data = motor_speeds_str;
  rcl_publish(&motor_speeds_publisher, &motor_speeds_msg, NULL);
}


// Callback function for handling pwml messages
void pwml_callback(const void *msg_recv)
{
  const std_msgs__msg__Int32 *received_data = (const std_msgs__msg__Int32 *)msg_recv;
  int32_t pwml = received_data->data;
  setMotorSpeeds(pwml, 0, 0, pwml); // Adjust motor speeds based on received PWM left value
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
  set_microros_serial_transports(Serial);
  delay(2000);  
  
  Wire.begin(33, 32); // sda= 33 /scl= 32 for sense and 22, 21 for ttgo_motor controller
  initMotorController();
  allocator = rcl_get_default_allocator();


  // Initialize micro-ROS
  // Set up support and allocator
  rclc_support_init(&support, 0, NULL, &allocator);

  // Set up executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  // Set up ROS node
  rclc_node_init_default(&node, "motor_control_node", "", &support);

  // Set up pwml subscription
  rclc_subscription_init_default(
      &pwml_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/pwml");

  // Add pwml subscription to the executor
  rclc_executor_add_subscription(
      &executor,
      &pwml_subscription,
      &pwml_msg,
      &pwml_callback,
      ON_NEW_DATA);

  // Set up PWMR subscription
  rclc_subscription_init_default(
      &pwmr_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/pwmr");

  // Add PWMR subscription to the executor
  rclc_executor_add_subscription(
      &executor,
      &pwmr_subscription,
      &pwmr_msg,
      &pwmr_callback,
      ON_NEW_DATA);

  rclc_publisher_init_default(
      &motor_speeds_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/motor_speeds");
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