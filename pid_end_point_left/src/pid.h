
#ifndef PID_H
#define PID_H
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class pid {     
    
    public:    

        pid();
        void begin(float p, float i, float d, int pwd_pin, int PWM_direction, int enc_pin);

        void IRAM_ATTR ISR_enc(){
            uint8_t r = digitalRead(A_enc_pin);
            if (xQueueSendFromISR( cola_enc , &r ,NULL) != pdTRUE)
            {
                printf("Error de escritura en la cola cola_enc \n");
            }
        }

        int32_t getEncoder() {
            return ang_cnt;
        }
        float getVelocity() {
            return v_medida;
        }

        void setPoint(float rad) {
            setpoint = rad;
            if (rad != 0) {
              
              start_stop = true;}
        }
        float getSetpoint() {
            return setpoint;
        }
       
    private:
        void task_enc();
        static pid* instance ;
        int num = 0;
        float wheel_radius = 0.044;
        float min_value_volt = 5.0;
        int taskCore = 1;
        void config_enc();
        static void loopcontr(void* _this);
        static void encoder(void* _this);
        static void medidas(void* _this);

        void config_PWM();
        void clean();
        void excita_motor(float v_motor);
        void task_loopcontr();
        
        void task_medidas();
        xQueueHandle cola_enc;
    
        float vel;
        bool start_stop;
        float Akpi=0;
        float Akp=0;
        float error_2=0; // e(t-2)
        float error_1 = 0; // e(t-1)
        float error_0 = 0; // e(t)
        float output = 0;  
        float A0d=0;
        float A1d=0;
        float A2d=0;
        float N=4;  // Filter level
        float tau = 0; // IIR filter time constant
        float alpha = 0;
        float d0 = 0;
        float d1 = 0;
        float fd0 = 0;
        float fd1 = 0;
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float setpoint = 0;
        float dt = 0;
        float z_i = 0;
        float z_d = 0;
        float Tiuser = 0;
        float Tduser = 0;
        float Ti = 0;
        float Td = 0;
        uint32_t pwmfreq = 1000; // 1KHz
        const uint8_t pwmChannel = 0;
        const uint8_t pwmresolution = 8;
        const int PWM_Max = pow(2,pwmresolution)-1; //
        uint8_t PWM_Pin = 0; 
        uint8_t PWM_direction =0;
        uint8_t A_enc_pin = 0;
        const float rpm_to_radians = 0.10471975512;
        const float rad_to_deg = 57.29578;
        int32_t ang_cnt = 0;
        float pwm_volt = 0;
        int32_t pwm_motor = 0;
        float v_medida = 0;     // Valor medido de angulo o velocidad -----------------
        float a_medida = 0;
        float ref_val = 0;    
        int direccion = 0;
        int direccion_ant = 0;  
        float error_anterior = 0;
        int ACTIVA_P1C_MED_ANG = 0;
        float da = 0;
        float anterior = 0;
};

#endif