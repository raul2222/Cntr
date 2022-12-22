#include <Arduino.h>
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#include <WiFi.h>
#endif
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <ArduinoJson.h>
#include <geometry_msgs/msg/twist.h>
#include <SoftwareSerial.h>
#include <geometry_msgs/msg/point.h>
#include <HardwareSerial.h>


#define RXD1 9
#define TXD1 10
#define RXD2 16
#define TXD2 17
// Right and Left
HardwareSerial SERIAL_RIGHT(2);
HardwareSerial SERIAL_LEFT(1);
#define MYPORT_TX 12
#define MYPORT_RX 13
float enc_r=0;
float enc_l=0;
SoftwareSerial myPort;


rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg_encoder;
geometry_msgs__msg__Twist msg;
rcl_subscription_t subscriber;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer_l;
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;
#define LED_PIN 2

float LINEAR_X = 0;
float ANGULAR_Z = 0;

//#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
//#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


void timer_callback_r(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {     
      String str = "";
      //String str2 = "";
    
      while(SERIAL_RIGHT.available() > 0){
        
        str = SERIAL_RIGHT.readStringUntil('\n');
        StaticJsonDocument<30> docr;
        DeserializationError error = deserializeJson(docr, str);

        if(enc_r != docr["e_r"] and !error) {

          enc_r = docr["e_r"];
          msg_encoder.linear.x=enc_r;
          //myPort.println(str);
        }
      }

      /*if(SERIAL_LEFT.available() > 0){
        
        str2 = SERIAL_LEFT.readStringUntil('\n');
        StaticJsonDocument<30> doc;
        DeserializationError error2 = deserializeJson(doc, str2);

        if(enc_l != doc["e_l"] and !error2) {

          enc_l = doc["e_l"];
          msg_encoder.linear.y=enc_l;
          //myPort.println(str2);
        }
      }*/
      rcl_publish(&publisher, &msg_encoder, NULL);
  }
}


void timer_callback_l(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {     
      //String str = "";
      String str2 = "";
    
      /*if(SERIAL_RIGHT.available() > 0){
        
        str = SERIAL_RIGHT.readStringUntil('\n');
        StaticJsonDocument<30> docr;
        DeserializationError error = deserializeJson(docr, str);

        if(enc_r != docr["e_r"] and !error) {

          enc_r = docr["e_r"];
          msg_encoder.linear.x=enc_r;
          //myPort.println(str);
        }
      }*/

      while(SERIAL_LEFT.available() > 0){
        
        str2 = SERIAL_LEFT.readStringUntil('\n');
        StaticJsonDocument<30> doc;
        DeserializationError error2 = deserializeJson(doc, str2);

        if(enc_l != doc["e_l"] and !error2) {

          enc_l = doc["e_l"];
          msg_encoder.linear.y=enc_l;
          //myPort.println(str2);
        }
      }
      rcl_publish(&publisher, &msg_encoder, NULL);
  }
}


float radius = 0.044;
float b = 0.32;
//twist message cb
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    // if velocity in x direction is 0 turn off LED, if 1 turn on LED
    digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);

    float  linear = msg->linear.x;
    float angular = msg->angular.z;
    float  leftwheel =  (linear - angular * b / 2.0) / radius;  // rad /s
    float rightwheel = (linear + angular * b / 2.0) / radius;
    StaticJsonDocument<30> docl;
    StaticJsonDocument<30> docr;
    docl["l"] = leftwheel;
    docr["r"] = rightwheel;
  
    serializeJson(docr, SERIAL_RIGHT);
    SERIAL_RIGHT.println();
  
    serializeJson(docl, SERIAL_LEFT);
    SERIAL_LEFT.println();
    LINEAR_X = msg->linear.x;
    ANGULAR_Z = msg->angular.z;
    
    /*myPort.print(LINEAR_X);
    myPort.print(",");
    myPort.println(ANGULAR_Z);
    myPort.print(",");
    myPort.println(leftwheel);
    myPort.print(",");
    myPort.println(rightwheel);*/
}



void setup() {
    
    //Serial.begin(115200); 
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
  byte local_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  IPAddress local_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 113);
  size_t agent_port = 8888;

  set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) || defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA)
  IPAddress agent_ip(192, 168, 1, 67);
  size_t agent_port = 8888;

  char ssid[] = "MOVISTAR_3D68";
  char psk[]= "12345678";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
  rmw_uros_set_custom_transport(
    MICROROS_TRANSPORTS_FRAMING_MODE,
    NULL,
    platformio_transport_open,
    platformio_transport_close,
    platformio_transport_write,
    platformio_transport_read
  );
#else
#error "No transport defined"
#endif

    myPort.begin(57600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
    if (!myPort) { // If the object did not initialize, then its configuration is invalid
      Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
      while (1) { // Don't continue with invalid configuration
        delay (1000);
      }
    } 
      
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  
    SERIAL_RIGHT.begin(57600, SERIAL_8N1, RXD2, TXD2); 
    SERIAL_LEFT.begin(57600, SERIAL_8N1, RXD1, TXD1); 
    delay(20);
  

  
  /*
    if(xTaskCreatePinnedToCore( task_enc_r , "task_enc_r", 2048, NULL, 1, NULL,0) != pdPASS){
      exit(-1);
    }*/
    
    state = WAITING_AGENT;
}



bool create_entities(){
  
 
    allocator = rcl_get_default_allocator();
  
     //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  
      // create subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/mobile_base_controller/cmd_vel_unstamped"));
  
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/pid_wheels_enc"));
  
        const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback_r));
    
    RCCHECK(rclc_timer_init_default(
        &timer_l,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback_l));
  
    // create executor
    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer_l));

    StaticJsonDocument<30> docl;
    StaticJsonDocument<30> docr;
    docl["l"] = 0;
    docr["r"] = 0;
    serializeJson(docr, SERIAL_RIGHT);
    SERIAL_RIGHT.println();
  
    serializeJson(docl, SERIAL_LEFT);
    SERIAL_LEFT.println();

    return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor_sub);
  rclc_executor_fini(&executor_pub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void loop() {
  //delay(100);
  
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(100, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
            rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(50));
            rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(50));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}




void task_enc_r(void* _this){
  while(1){
      /*if(SERIAL_RIGHT.available() > 0){
         String str = SERIAL_RIGHT.readStringUntil('\n');
         StaticJsonDocument<30> docr;
         DeserializationError error = deserializeJson(docr, str);
         if(enc_r != docr["e_r"]) {
            //rcl_publish(&publisher, &msg_encoder, NULL);
         }
         //msg_encoder.x=enc_r;
         //rcl_publish(&publisher, &msg_encoder, NULL);
         enc_r = docr["e_r"]; // to pub
         myPort.print(enc_r);
         myPort.println(",");
      }*/
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }
      
}



void error_loop(){
  int i = 0;
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    if(i++ > 50 ) ESP.restart();
  }
}