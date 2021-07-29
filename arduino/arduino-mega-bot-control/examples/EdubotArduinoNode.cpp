#define ROS_CYCLE_TIME_PERIOD 10 // millisecond
#define ARDUINO_ID "U1/"

#include <Arduino.h>

#include <ros.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <Encoder.h>

#define N 5

#define PWM_PIN_L 10
#define PWM_PIN_R 9
#define DIR_PIN_L 6
#define DIR_PIN_R 7

// Declaring ROS Objects

// ROS Node Handler
ros::NodeHandle node_handle;

// Declaring callbacks for subscribers
void target_PWM_L_callback(const std_msgs::UInt8 &msg);
void target_PWM_R_callback(const std_msgs::UInt8 &msg);
void target_direction_L_callback(const std_msgs::Bool &msg);
void target_direction_R_callback(const std_msgs::Bool &msg);

// Declaring Subscribers
ros::Subscriber<std_msgs::UInt8> target_PWM_L_sub(ARDUINO_ID "L/P", &target_PWM_L_callback);
ros::Subscriber<std_msgs::UInt8> target_PWM_R_sub(ARDUINO_ID "R/P", &target_PWM_R_callback);
ros::Subscriber<std_msgs::Bool> target_direction_L_sub(ARDUINO_ID "L/D", &target_direction_L_callback);
ros::Subscriber<std_msgs::Bool> target_direction_R_sub(ARDUINO_ID "R/D", &target_direction_R_callback);

// Declaring message variables for publishers

// Encoder reading matrices
std_msgs::Int16MultiArray encoder_matrix_L_msg, encoder_matrix_R_msg;

// A template for dimension type objects
// to be shared by both encoder reading matrix messages
std_msgs::MultiArrayDimension encoder_matrix_dimension[1];

// Declaring Publishers
ros::Publisher encoder_matrix_L_pub(ARDUINO_ID "L/E", &encoder_matrix_L_msg);
ros::Publisher encoder_matrix_R_pub(ARDUINO_ID "R/E", &encoder_matrix_R_msg);

// Matrices to store and process encoder readings
int16_t encoder_matrix_L[N];
int16_t encoder_matrix_R[N];

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoder_L(2, 4);
Encoder encoder_R(3, 5);
//   avoid using pins with LEDs attached

// Variable to check last ros cycle time
unsigned long prev_ros_cylce_time;

// Variable to iterate over arrays
uint8_t i;

// Declaring functions for initialization
void init_encoder_matrix_update_timer();
void init_encoder_matrices();
void init_ros_pubs();
void init_ros_subs();
void init_pins();

void setup()
{
  // Initializing the ros node
  node_handle.initNode();
  node_handle.logdebug("ROS Node initialized");

  init_encoder_matrix_update_timer();
  node_handle.logdebug("Timers initialized");

  init_encoder_matrices();
  init_ros_pubs();
  init_ros_subs();
  init_pins();

  node_handle.loginfo("ROS Node started successfully"); 
}

void loop()
{
  if (millis() - prev_ros_cylce_time < ROS_CYCLE_TIME_PERIOD)
  {
    encoder_matrix_L_pub.publish(&encoder_matrix_L_msg);
    encoder_matrix_R_pub.publish(&encoder_matrix_R_msg);

    node_handle.spinOnce();

    prev_ros_cylce_time = millis();
  }
}

void init_encoder_matrix_update_timer()
{
  // stop interrupts
  cli();

  // set timer2 interrupt at 8kHz
  
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2  = 0; // initialize counter value to 0
  
  // set compare match register for 8khz increments
  OCR2A = 249;  // = (16*10^6) / (8000*8) - 1 (must be <256)
  
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS22);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  //allow interrupts
  sei();
}


void init_encoder_matrices()
{
  // creating only one instance of MultiArrayDimension
  // to be shared by both left and right encoder reading matrices
  encoder_matrix_dimension[0].size = N;
  encoder_matrix_dimension[0].stride = N;

  encoder_matrix_L_msg.layout.dim_length = 1;
  encoder_matrix_L_msg.layout.dim = encoder_matrix_dimension;

  encoder_matrix_L_msg.data_length = N;
  encoder_matrix_L_msg.data = encoder_matrix_L;

  encoder_matrix_R_msg.layout.dim_length = 1;
  encoder_matrix_R_msg.layout.dim = encoder_matrix_dimension;

  encoder_matrix_R_msg.data_length = N;
  encoder_matrix_R_msg.data = encoder_matrix_R;  
}

void init_ros_pubs()
{
  node_handle.advertise(encoder_matrix_L_pub);
  node_handle.advertise(encoder_matrix_R_pub);
}

void init_ros_subs()
{
  node_handle.subscribe(target_PWM_L_sub);
  node_handle.subscribe(target_PWM_R_sub);
  node_handle.subscribe(target_direction_L_sub);
  node_handle.subscribe(target_direction_R_sub);
}

void init_pins()
{
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);

  digitalWrite(DIR_PIN_L, HIGH);
  digitalWrite(DIR_PIN_R, LOW);

  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);
}

void target_PWM_L_callback(const std_msgs::UInt8 &msg) { analogWrite(PWM_PIN_L, msg.data); }
void target_PWM_R_callback(const std_msgs::UInt8 &msg) { analogWrite(PWM_PIN_R, msg.data); }
void target_direction_L_callback(const std_msgs::Bool &msg) { digitalWrite(DIR_PIN_L, msg.data); }
void target_direction_R_callback(const std_msgs::Bool &msg) { digitalWrite(DIR_PIN_R, msg.data); }

ISR(TIMER2_COMPA_vect)
{
  for (i=N-1; i>0; i--) 
  {
    encoder_matrix_L[i-1] = encoder_matrix_L[i];
    encoder_matrix_R[i-1] = encoder_matrix_R[i];
  }

  encoder_matrix_L[0] = encoder_L.read();
  encoder_matrix_R[0] = encoder_R.read();  
}
