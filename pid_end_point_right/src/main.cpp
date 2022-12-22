
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include "pid.h"
pid my_pid;

#define RXD2 16
#define TXD2 17
#include <HardwareSerial.h>
// RIGHT
HardwareSerial MySerial2(2);

float VEL = 0;

void task_vel(void* _this){
  while (1) {
      if(MySerial2.available() > 0){ ///  -->TWIST-->RAD/S-->
          String str = MySerial2.readStringUntil('\n');
          StaticJsonDocument<30> doc;
          DeserializationError error = deserializeJson(doc, str);
          //  Change for correct wheel
          VEL = doc["r"];
          if(!error){
            my_pid.setPoint(VEL);
          }
          Serial.println(VEL);
      }
      vTaskDelay(20 / portTICK_PERIOD_MS);
  } 
}


void task_encore_data(void* _this)
{
  while (1) {
      StaticJsonDocument<30> docr;
      docr["e_r"] = my_pid.getEncoder();

      serializeJson(docr, MySerial2);
      MySerial2.println();
        
      vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}


void setup()
{
  MySerial2.begin(57600, SERIAL_8N1, RXD2, TXD2); 
  delay(1000);
  Serial.begin(115200); 
  /******** TEST ***********/
  Serial.println("Start test");

  my_pid.begin(0.55 ,0.018 ,2 , 32, 4, 35);
  

  
  if(xTaskCreatePinnedToCore( task_vel , "task_vel", 2048, NULL, 1, NULL,1) != pdPASS){
    exit(-1);
  }
    
  if(xTaskCreatePinnedToCore( task_encore_data , "task_encore_data", 2048, NULL, 1, NULL,0) != pdPASS){
    exit(-1);
  }

  my_pid.setPoint(0);
  
}



void loop()
{

}