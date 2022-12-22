#include "pid.h"
#include <FunctionalInterrupt.h>

// Parametros cola de la interrupcion del encoder ///////////////////////////////////////
#define TAM_COLA_I 1024 /*num mensajes*/
#define TAM_MSG_I 1 /*num caracteres por mensaje*/

// TIEMPOS
#define BLOQUEO_TAREA_LOOPCONTR_MS 10 
#define BLOQUEO_TAREA_MEDIDA_MS 500

// Outside of class
pid *pointerToClass; // declare a pointer to testLib class

static void IRAM_ATTR outsideInterruptHandler(void) { // define global handler
  pointerToClass->ISR_enc(); // calls class member handler
}
#define LED_PIN LED_BUILTIN

pid::pid(){

    start_stop=false;
    cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
    if(cola_enc == NULL){
        exit(-1);
    }
   
}

void pid::begin(float p, float i, float d, int pwm_pin, int pwm_direction, int enc_pin) {
  
    Kp = p;
    Ki=i;
    Kd=d;
    PWM_Pin = pwm_pin;
    PWM_direction = pwm_direction;
    A_enc_pin = enc_pin;
    dt = (BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
    config_PWM();
    config_enc();
    pointerToClass = this;
   
    attachInterrupt(A_enc_pin, outsideInterruptHandler, CHANGE);
    
    if(xTaskCreatePinnedToCore( this->loopcontr , "task_loopcontr", 2048, this, 4, NULL,0) != pdPASS){
       exit(-1);
    }
    if(xTaskCreatePinnedToCore( this->medidas , "task_medidas", 2048, this, 1, NULL,1) != pdPASS){
       exit(-1);
    }
    if(xTaskCreatePinnedToCore( this->encoder , "task_enc", 2048, this, 4, NULL,0) != pdPASS){
       exit(-1);
    }
}

void pid::loopcontr(void* _this){
    static_cast<pid*>(_this)->task_loopcontr();
    
}
void pid::task_loopcontr() {

    while(1) {    
        if(start_stop == true){
            Akpi=Kp+(Ki*dt);
            Akp = -Kp;
            A0d = Kd/dt;
            A1d = (-2.0)*(Kd/dt);
            A2d = Kd/dt;
            Kd == 0 ? tau = 0 : tau = Kd/(Kp*N); // IIR filter time constant  
            isinf(dt / (2*tau)) ? alpha = 0 : alpha = dt / (2*tau);

            if (ACTIVA_P1C_MED_ANG == 0){
                // rps_vel
                v_medida = (ang_cnt * 2.0 * PI) / 1008.0;
                da = v_medida - anterior;
                anterior = v_medida;
                v_medida = da / (BLOQUEO_TAREA_LOOPCONTR_MS / 1008.0); // rad/s
                //v_medida = v_medida / (2.0 * PI); // rps
                //v_medida = v_medida * radius_whell; // m/s
            } else {
                v_medida = (ang_cnt * 360.0) / 1008.0;  // Calculo de angulo
                //no_error_motor_break();
            }

            error_2 = error_1;
            error_1 = error_0;
            error_0 = setpoint - v_medida;
            // PI
            output = output+(Akpi*error_0)+(Akp*error_1);
            // Filtered D
            if(alpha !=0) {
                d1 = d0;
                d0 = (A0d * error_0) + (A1d * error_1) + (A2d * error_2);
                fd1 = fd0;
                fd0 = ((alpha) / (alpha + 1)) * (d0 + d1) - ((alpha - 1) / (alpha + 1)) * fd1;
                output = output + fd0;  
            }
            if (abs(output) > 12.0 and output > 0) output = 12 ; // min voltage value for dc-motor
            if (abs(output) < 4.7 and ref_val > 0) output = 4.7 ;  // min voltage value for dc-motor 
            if (abs(output) > 12.0 and output < 0) output = -12 ;
            if (abs(output) < 4.7 and ref_val < 0) output = -4.7 ;
            if(setpoint == 0) {
                start_stop =false;
                excita_motor(0);
                //clean(); 
            } else{ 
                excita_motor(output);
            }
        } else {

            //clean();
        }
        // Activacion de la tarea cada 0.01s
        vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
    }
}


void pid::encoder(void* _this){
    static_cast<pid*>(_this)->task_enc(); 
}

void pid::task_enc() {
    // Declaracion de variables locales
    uint8_t r ;
    uint8_t anterior ;
    while(1){
        // Espera a leer los datos de la cola
        if (xQueueReceive( cola_enc , &r ,(TickType_t) portMAX_DELAY) == pdTRUE){
            if(r != anterior){
              if(output != 0){
                if(output > 0){
                  ang_cnt++;
                } else {
                  ang_cnt--;
                }
              }
            }
            anterior = r;
        } else {
            printf("Error de lectura de la cola cola_enc \n");
        }
    }
}  

void pid::medidas(void* _this){
    static_cast<pid*>(_this)->task_medidas();
}

void pid::task_medidas()
{
    while (1) {
        if (start_stop == 1) {
          /*
            // Mostrar medidas de angulo y velocidad del motor
            if ( ACTIVA_P1C_MED_ANG == 1 ) { // Medida de angulo
                //a_medida = (ang_cnt * 360) / 1200;
                Serial.print("M:");
                Serial.print(a_medida,3);
                Serial.print(" A:");
                Serial.print(setpoint, 2);

            } else { // Medida de velocidad
                Serial.print("Med:");
                Serial.print(v_medida,3);
                Serial.print(", Ref:");
                Serial.print(setpoint, 2);
            }
            Serial.print(" V:");
            Serial.print(output, 6);
            Serial.print(" Kp:");
            Serial.print(Kp, 6);
            Serial.print(" Ki:");
            Serial.print(Ki, 6);
            Serial.print(" Kd:");
            Serial.print(Kd, 6);
            Serial.print(" z_i:");
            Serial.print(z_i, 3);
            Serial.print(" z_d:");
            Serial.print(z_d, 3);
            Serial.println(" ");
            */
        } 
    // Activacion de la tarea cada 1s
    vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS / portTICK_PERIOD_MS);
  }
}

void pid::clean(){
    excita_motor(0);
    error_2 = 0; 
    error_1 = 0;
    error_0 = 0;
    output = 0;
    v_medida = 0;
    da = 0;
    anterior = 0;
    d1 = 0;
    d0 = 0;
    fd1 = 0;
    fd0 = 0;
}

void pid::excita_motor(float v_motor){
    // Obtención de la dirección
    if(v_motor > 0){
        direccion = 1;
    }else{
        direccion = 0;
    }
    
    if(direccion_ant != direccion){  //("Cambio de sentido");
    }
    if(v_motor > 0){    //Serial.println("Hacia adelante");
        digitalWrite(PWM_direction, LOW); // el pin de direccion
    }
    if(v_motor < 0){    //("Hacia atras");
        v_motor = abs(v_motor); // valor en positivo del voltaje el cambio de direccion lo hacen las variables
        digitalWrite(PWM_direction, HIGH);
    }
    direccion_ant = direccion;
    // Calcula y limita el valor de configuración del PWM
    int dutyCycle = (int) ((v_motor * 255)/12);
    // El valor de excitación debe estar entro 0 y PWM_Max
    if(dutyCycle >= PWM_Max){
        dutyCycle = PWM_Max;
    }
  if(dutyCycle <= 0){
    dutyCycle = 0;
  }
  // Excitacion del motor con PWM
  ledcWrite(0, dutyCycle);
}  

void pid::config_PWM(){
  // Configuracion de pines de control PWM
  pinMode(PWM_direction, OUTPUT);
  digitalWrite(PWM_direction, LOW);
  // Configuracion LED PWM 
  ledcSetup(pwmChannel, pwmfreq, pwmresolution);
  // Asignar el controlador PWM al GPIO
  ledcAttachPin(PWM_Pin, 0);
}  

void pid::config_enc(){
  pinMode(A_enc_pin, INPUT);
}
