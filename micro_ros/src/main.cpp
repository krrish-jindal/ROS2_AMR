//    correction base

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int16.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/string.h>
#include <WiFi.h>

int16_t received_pwml_data = 0; // Global variable to store received pwml data
int16_t received_pwmr_data = 0; // Global variable to store received pwmr data

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t pwml_subscription;
rcl_subscription_t pwmr_subscription;
rcl_publisher_t motor_speeds_publisher;
std_msgs__msg__String motor_speeds_msg;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

char ssid[] = "lg";
char psk[] = "krrish2002";
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x40);

#define IFDAC_ENABLED 1
#define FRONTLEFT 0
#define FRONTRIGHT 1
#define BACKRIGHT 2
#define BACKLEFT 3

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(4);

void initMotorController()
{
  if (IFDAC_ENABLED)
  {
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

void setMotorSpeed(int motor, int spd)
{
  switch (motor)
  {
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

void setMotorSpeeds(int frontLeftSpeed, int frontRightSpeed, int backRightSpeed, int backLeftSpeed)
{
  setMotorSpeed(FRONTLEFT, -frontLeftSpeed);
  setMotorSpeed(FRONTRIGHT, -frontRightSpeed);
  setMotorSpeed(BACKRIGHT, backRightSpeed);
  setMotorSpeed(BACKLEFT, backLeftSpeed);

  char motor_speeds_str[50]; // Adjust the buffer size based on your needs
  snprintf(motor_speeds_str, sizeof(motor_speeds_str), "%d,%d,%d,%d", frontLeftSpeed, frontRightSpeed, backRightSpeed, backLeftSpeed);

  motor_speeds_msg.data.data = motor_speeds_str;
  rcl_ret_t publish_status = rcl_publish(&motor_speeds_publisher, &motor_speeds_msg, NULL);
  if (publish_status != RCL_RET_OK)
  {
    Serial.print("Failed to publish motor speeds message. Error code: ");
    Serial.println(publish_status);
  }
}

// Callback function for handling pwml messages
void pwml_callback(const void *msg_recv)
{
  const std_msgs__msg__Int16 *received_data = (const std_msgs__msg__Int16 *)msg_recv;
  received_pwml_data = received_data->data;
}

// Callback function for handling PWMR messages
void pwmr_callback(const void *msg_recv)
{
  const std_msgs__msg__Int16 *received_data = (const std_msgs__msg__Int16 *)msg_recv;
  received_pwmr_data = received_data->data;
}

void destroy_entities()
{
  setMotorSpeeds(0, 0, 0, 0);

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&motor_speeds_publisher, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS
  // Set up support and allocator
  rclc_support_init(&support, 0, NULL, &allocator);

  // Set up executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  // Set up ROS node
  rclc_node_init_default(&node, "motor_control_node", "", &support);

  rclc_subscription_init_default(
      &pwml_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "/pwml");

  rclc_subscription_init_default(
      &pwmr_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "/pwmr");

  rclc_publisher_init_default(
      &motor_speeds_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/motor_speeds");

  rclc_executor_add_subscription(
      &executor,
      &pwml_subscription,
      &received_pwml_data,
      &pwml_callback,
      ON_NEW_DATA);

  rclc_executor_add_subscription(
      &executor,
      &pwmr_subscription,
      &received_pwmr_data,
      &pwmr_callback,
      ON_NEW_DATA);
  return true;
}

void setup()
{
  IPAddress agent_ip(192, 168, 9, 151);
  size_t agent_port = 8888;

  Serial.begin(115200);


  //         TO ENABLE SERIAL

  // set_microros_serial_transports(Serial);



  //          TO ENABLE WIFI
  
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    delay(1000);
  }

  Serial.println("Connected to wifi");

  state = WAITING_AGENT;

  Wire.begin(33, 32);

  initMotorController();
}

void loop()
{
  Serial.println("state");

  Serial.println(state);

  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      Serial.println("CONNECTION ESTABLISHED");
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }

  if (state == AGENT_CONNECTED)
  {
    Serial.println("START");
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    setMotorSpeeds(received_pwml_data, received_pwmr_data, received_pwml_data, received_pwmr_data);
  }
  else
  {
    Serial.println("END");
  }
}
