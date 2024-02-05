//    correction base

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int16.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <WiFi.h>
#include "soc/rtc_io_reg.h"
#include <std_msgs/msg/string.h>


int16_t received_pwml_data = 0; // Global variable to store received pwml data
int16_t received_pwmr_data = 0; // Global variable to store received pwmr data

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t pwml_subscription;
rcl_subscription_t pwmr_subscription;
rcl_publisher_t encoder_data__publisher;
std_msgs__msg__Int32MultiArray encoder_msg;

rcl_publisher_t motor_data_publisher;
std_msgs__msg__String motor_data_msg;

int32_t encoderdata[10];  
float error[5];  

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


//   ##################################################################

const int HALLSEN_A = 14; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B = 27; // Hall sensor A connected to pin 3 (external interrupt)

const int HALLSEN_A2 = 25; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B2 = 26; // Hall sensor A connected to pin 3 (external interrupt)

const int HALLSEN_A3 = 18; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B3 = 19; // Hall sensor A connected to pin 3 (external interrupt)

const int HALLSEN_A4 = 13; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B4 = 17; // Hall sensor A connected to pin 3 (external interrupt)

volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
volatile long encoderValue3 = 0;
volatile long encoderValue4 = 0;

volatile long lastEncoderValue1 = 0;
volatile long lastEncoderValue2 = 0;
volatile long lastEncoderValue3 = 0;
volatile long lastEncoderValue4 = 0;



int16_t ticks_per_rev = 280;
float ticks_per_meter = 891.26;
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL = 100; 

int rpm1 = 0;
int rpm2 = 0;
int rpm3 = 0;
int rpm4 = 0;

int PWM_MIN = 0;   // Minimum PWM value
int PWM_MAX = 240; // Maximum PWM value

const float RPM_MIN = 0.0;   // Minimum RPM value
const float RPM_MAX = 200.0; // Maximum RPM value


float error1 = 0.0;
float error2 = 0.0;
float error3 = 0.0;
float error4 = 0.0;

float kp =2.0;
float kd =0.1;
float ki =0.1;


#define ENCODEROUTPUT 1
#define GEARRATIO 50

//   ##################################################################

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

  pinMode(HALLSEN_A, INPUT);
  pinMode(HALLSEN_B, INPUT);
  pinMode(HALLSEN_A2, INPUT);
  pinMode(HALLSEN_B2, INPUT);
  pinMode(HALLSEN_A3, INPUT);
  pinMode(HALLSEN_B3, INPUT);
  pinMode(HALLSEN_A4, INPUT);
  pinMode(HALLSEN_B4, INPUT);


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

   
  setMotorSpeed(FRONTLEFT, -min(frontLeftSpeed, PWM_MAX));
  setMotorSpeed(FRONTRIGHT, -min(frontRightSpeed, PWM_MAX));
  setMotorSpeed(BACKRIGHT, min(backRightSpeed, PWM_MAX));
  setMotorSpeed(BACKLEFT, min(backLeftSpeed, PWM_MAX));

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





void updateEncoder1() {


  if (digitalRead(HALLSEN_A) == digitalRead(HALLSEN_B)) {
    encoderValue1++;
  } else {
    encoderValue1--;
  }
}

void updateEncoder2() {

  if (digitalRead(HALLSEN_A2) == digitalRead(HALLSEN_B2)) {
    encoderValue2--;
  } else {
    encoderValue2++;
  }
}

void updateEncoder3() {
  if (digitalRead(HALLSEN_A3) == digitalRead(HALLSEN_B3)) {
    encoderValue3--;
  } else {
    encoderValue3++;
  }
}

void updateEncoder4() {
  if (digitalRead(HALLSEN_A4) == digitalRead(HALLSEN_B4)) {
    encoderValue4--;
  } else {
    encoderValue4++;
  }
}



float pid(int desire, int actual, float min_val_, float max_val_)
{
    static double integral_ = 0; // Make it static to preserve its value between function calls
    static double prev_error_ = 0; // Make it static to preserve its value between function calls

    double error = desire - actual;
    integral_ += error;
    double derivative_ = error - prev_error_;

    if (desire == 0 && error == 0)
    {
        integral_ = 0;
        derivative_ = 0;
    }

    double pid = (kp * error) + (ki * integral_) + (kd * derivative_);
    prev_error_ = error;

    char motor_data_str[30]; // Adjust the buffer size based on your needs
    snprintf(motor_data_str, sizeof(motor_data_str), "%d,%d", error, pid);

    motor_data_msg.data.data = motor_data_str;
    rcl_publish(&motor_data_publisher, &motor_data_msg, NULL);
  

    return constrain(pid, min_val_, max_val_);
}

void EncoderInit() {
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder1, CHANGE);

  attachInterrupt(digitalPinToInterrupt(HALLSEN_A2), updateEncoder2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(HALLSEN_A3), updateEncoder3, CHANGE);

  attachInterrupt(digitalPinToInterrupt(HALLSEN_A4), updateEncoder4, CHANGE);
}


void destroy_entities()
{
  setMotorSpeeds(0, 0, 0, 0);

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&encoder_data__publisher, &node);
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
      &encoder_data__publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/encoderdata");

  rclc_publisher_init_default(
      &motor_data_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/motor_data");


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


int pwmToRPM(int pwm, int minPWM, int maxPWM, int minRPM, int maxRPM) {
    // Ensure that pwm is within the specified range
    pwm = constrain(pwm, minPWM, maxPWM);

    // Map pwm to RPM using linear interpolation
    return map(pwm, minPWM, maxPWM, minRPM, maxRPM);
}



void setup()
{
  IPAddress agent_ip(192, 168, 75, 55);
  size_t agent_port = 8888;

  Serial.begin(115200);


  //         TO ENABLE SERIAL

  set_microros_serial_transports(Serial);



  //          TO ENABLE WIFI
  
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   Serial.print("Attempting to connect to SSID: ");
  //   Serial.println(ssid);
  //   // WiFi.begin(ssid,psk);
  //   set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  //   delay(1000);
  // }

  Serial.println("Connected to wifi");

  state = WAITING_AGENT;

  Wire.begin(33, 32);

  encoderValue1 = 0;
  encoderValue2 = 0;
  encoderValue3 = 0;
  encoderValue4 = 0;
  encoder_msg.data.data = encoderdata;
  encoder_msg.data.size = 10;  

  initMotorController();
  EncoderInit();
}


void loop()
{ 
  Serial.println(state);
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastUpdateTime;

  if (elapsedTime >= UPDATE_INTERVAL)
  {
    float timeInSeconds = elapsedTime / 1000.0; // Convert time to seconds

    // Calculate RPM for each motor based on the change in encoder values
    rpm1 = (encoderValue1 - lastEncoderValue1) * 60 / (ticks_per_rev * timeInSeconds);  // LEFT BACK
    rpm2 = (encoderValue2 - lastEncoderValue2) * 60 / (ticks_per_rev * timeInSeconds);  // RIGHT FRONT
    rpm3 = (encoderValue3 - lastEncoderValue3) * 60 / (ticks_per_rev * timeInSeconds);  // RIGHT BACK
    rpm4 = (encoderValue4 - lastEncoderValue4) * 60 / (ticks_per_rev * timeInSeconds);  // LEFT FRONT

    // Update last encoder values for the next calculation
    lastEncoderValue1 = encoderValue1;
    lastEncoderValue2 = encoderValue2;
    lastEncoderValue3 = encoderValue3;
    lastEncoderValue4 = encoderValue4;

    lastUpdateTime = currentTime;
  }




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



  float desiredRPM_L = RPM_MIN + (RPM_MAX - RPM_MIN) * (abs(received_pwml_data) - PWM_MIN) / (PWM_MAX - PWM_MIN);

    // Ensure the calculated RPM is within the desired range
    desiredRPM_L = constrain(desiredRPM_L, RPM_MIN, RPM_MAX);

  float desiredRPM_R = RPM_MIN + (RPM_MAX - RPM_MIN) * (abs(received_pwmr_data) - PWM_MIN) / (PWM_MAX - PWM_MIN);

    // Ensure the calculated RPM is within the desired range
    desiredRPM_R = constrain(desiredRPM_R, RPM_MIN, RPM_MAX);


//     FOOR MAP PWM_RPM

  // int desiredRPM_L = pwmToRPM(received_pwml_data, 0, 255, 0.0, 200);
  // int desiredRPM_R = pwmToRPM(received_pwmr_data, 0, 255, 0.0, 200);


  encoderdata[0]=encoderValue1;
  encoderdata[1]=encoderValue2;
  encoderdata[2]=encoderValue3;
  encoderdata[3]=encoderValue4;
  encoderdata[4]=desiredRPM_L;
  encoderdata[5]=desiredRPM_R;

  encoderdata[6]=rpm1;
  encoderdata[7]=rpm2;
  encoderdata[8]=rpm3;
  encoderdata[9]=rpm4;

  // ----------P-Controller------------//

  // error1=abs(abs(desiredRPM_L)-abs(rpm4));
  error1=pid(abs(desiredRPM_L),abs(rpm4),0.0,2.0);

  // error2=abs(abs(desiredRPM_R)-abs(rpm2));
  error2=pid(abs(desiredRPM_R),abs(rpm2),0.0,2.0);

  // error3=abs(abs(desiredRPM_L)-abs(rpm1));
  error3=pid(abs(desiredRPM_L),abs(rpm1),0.0,2.0);

  // error4=abs(abs(desiredRPM_R)-abs(rpm3));
  error4=pid(abs(desiredRPM_R),abs(rpm3),0.0,2.0);

  // -------------------------//



  encoder_msg.data.data = encoderdata;
  rcl_ret_t publish_status = rcl_publish(&encoder_data__publisher, &encoder_msg, NULL);
  if (publish_status != RCL_RET_OK)
  {
    Serial.print("Failed to publish motor speeds message. Error code: ");
    Serial.println(publish_status);
  }


  
  if (state == AGENT_CONNECTED)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    setMotorSpeeds(received_pwml_data, received_pwmr_data, received_pwml_data, received_pwmr_data);
  }
  else
  {
    Serial.println("END");
  }
}
