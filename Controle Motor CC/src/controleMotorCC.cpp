#include "controleMotorCC.h"
#include <Arduino.h>

#define MOTOR_PWM 5
#define MOTOR_R 18
#define MOTOR_L 19

#define LTs 26
bool countTs = true;

// Connstantes utilizadas no tempo continuo:
//double Kp = 0.667, Ki = 1, Kd = 0;
//double Kp = 1.18, Ki = 20, Kd = 0;
//double Kp = 1.5340, Ki = 26, Kd = 0;
double Kp = 1.0200, Ki = 30, Kd = 0;
//double Kp = 1.5, Ki = 30, Kd = 0;


// ===================== Declarando variáveis ====================== //

double error = 0, error1 = 0, error2 = 0;
double output = 0, output1 = 0, output2 = 0;

const double PAD_IN = 1000;
const double PAD_PWM = 21.25;

unsigned long tempofinal = 0;
float Ts = 10; //em ms


// ================================================================== //

void initMotorControle() {
    pinMode(MOTOR_R, OUTPUT);
    pinMode(MOTOR_L, OUTPUT);

    pinMode(LTs, OUTPUT); // Para medir Ts manualmente

    ledcSetup(0, 25000, 8); // canal 0 para MOTOR_R, 20kHz, 8 bits
    ledcSetup(1, 25000, 8); // canal 1 para MOTOR_L, 20kHz, 8 bits
    ledcAttachPin(MOTOR_R, 0);
    ledcAttachPin(MOTOR_L, 1);

    tempofinal = micros(); // atualizando tempo atual para o calculo de Ts
}

void controleMotor(float velocidade, float setpoint) {
    
    // ==================== Calcula Ts atual (em ms) ==================== //

    // utilizando micros para melhor resolução
    unsigned long agora = micros(); 
    
    unsigned long delta_us = agora - tempofinal;
    float delta_ms = delta_us/1000; //micro -> mili

    tempofinal = agora;

    // ================================================================== //

    // ============== Calculo de erro e utilização do PID ============== //

    countTs = !countTs;                 // inverte o estado
    digitalWrite(LTs, countTs);         // Para medir Ts

    double error = (setpoint - velocidade) / PAD_IN;

    //Serial.print(error, 6);
    //Serial.println();

    float Ts_s = Ts/1000; // Periodo de amostragem no tempo discreto (conversão para segundos)

    // Equação de diferenças:
    //output = output1 + (Kp + Kd/Ts_s)*error + (-Kp + Ki*Ts_s - 2*Kd/Ts_s)*error1 + (Kd/Ts_s)*error2;


    //PID Tustin:
    //output = output2 + (Kp + Ki*Ts_s/2.0)*error + (Ki*Ts_s)*error1 + (-Kp + Ki*Ts_s/2.0)*error2;

    //PI TUSTIN:
    output = output1 + (Kp + Ki*Ts_s/2.0)*error + (-Kp + Ki*Ts_s/2.0)* error1;

    // Limites para output:
    if (output > 12) output = 12;
    if (output < -12) output = -12;

    output1 = output;
    output2 = output1;
    error2 = error1;
    error1 = error;
    
    

    //Serial.print(output);
    //Serial.println();
    //if (true) {
        // Motor gira num sentido
    //   ledcWrite(0, 0);             // MOTOR_R pwm = 0
    //    ledcWrite(1, 100); // MOTOR_L pwm = output
    if (output > 0) {
        // Motor gira num sentido
        ledcWrite(0, 0);             // MOTOR_R pwm = 0
        ledcWrite(1, (int)(output*PAD_PWM)); // MOTOR_L pwm = output
    } else {
        // Motor gira no sentido oposto
        ledcWrite(1, 0);             // MOTOR_L pwm = 0
        ledcWrite(0, (int)(abs(output*PAD_PWM))); // MOTOR_R pwm = output
    }
}

void pararMotorControle() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);

    error = 0;
    error1 = 0;
    error2 = 0;

    output = 0;
    output1 = 0;
    output2 = 0;

    tempofinal = micros();
}