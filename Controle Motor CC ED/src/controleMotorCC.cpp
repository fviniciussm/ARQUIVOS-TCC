#include "controleMotorCC.h"
#include <Arduino.h>

// ===================== Definições de pinos ====================== //

#define MOTOR_PWM 5
#define MOTOR_R 18
#define MOTOR_L 19

// Pino de saída para medir Ts no osciloscópio
#define LTs 26
bool countTs = true;

// ================================================================== //

// ===================== Constantes do PID ======================== //

//constexpr double Kp = 1.0200;
//constexpr double Ki = 30.0;

constexpr double Kp = 1.000;
constexpr double Ki = 30.3;

constexpr double Kd = 0.0;

constexpr double PAD_IN  = 1000.0;
constexpr double PAD_PWM = 21.25;

constexpr float Ts   = 10.0f;
constexpr float Ts_s = Ts / 1000.0f;

constexpr double OUTPUT_MAX = 12.0;
constexpr double OUTPUT_MIN = -12.0;

// ================================================================== //

// ===================== Variáveis ================================ //

// Erros e saídas do controlador (estados para equação de diferenças)
double error = 0.0, error1 = 0.0, error2 = 0.0;
double output = 0.0, output1 = 0.0, output2 = 0.0;

// ================================================================== //

// ===================== Inicialização ============================ //

void initMotorControle() {
    pinMode(MOTOR_R, OUTPUT);
    pinMode(MOTOR_L, OUTPUT);

    pinMode(LTs, OUTPUT); // Para medir Ts manualmente

    // PWM do motor: 25 kHz, (canal 0 = MOTOR_R, canal 1 = MOTOR_L)
    ledcSetup(0, 25000, 8);
    ledcSetup(1, 25000, 8);
    ledcAttachPin(MOTOR_R, 0);
    ledcAttachPin(MOTOR_L, 1);
}

// ================================================================== //

// ===================== Controle do motor ======================== //

void controleMotor(float velocidade, float setpoint) {

    // Pino 26, LTs: no osciloscópio, o período entre bordas = período do loop
    countTs = !countTs;
    digitalWrite(LTs, countTs);         // Para medir Ts

    // Erro normalizado pela escala PAD_IN (setpoint e velocidade em RPM)
    double error = (setpoint - velocidade) / PAD_IN;

    // PI Tustin (forma incremental)
    output = output1 + (Kp + Ki * Ts_s / 2.0) * error + (-Kp + Ki * Ts_s / 2.0) * error1;

    // Limites para saida:
    if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;

    // Atualiza estados do controlador para a próxima amostra
    output1 = output;
    output2 = output1;
    error2 = error1;
    error1 = error;

    // ===================== Acionamento da ponte H ================== //
    //if (true) {
        // Motor gira num sentido
    //    ledcWrite(0, 0);                              // MOTOR_R pwm = 0
    //    ledcWrite(1, 255); 

    if (output > 0) {
        // Motor gira num sentido
        ledcWrite(0, 0);                              // MOTOR_R pwm = 0
        ledcWrite(1, (int)(output * PAD_PWM));          // MOTOR_L pwm = output
    } else {
        // Motor gira no sentido oposto
        ledcWrite(1, 0);                                // MOTOR_L pwm = 0
        ledcWrite(0, (int)(abs(output * PAD_PWM)));     // MOTOR_R pwm = output
    }
}

// ================================================================== //

// ===================== Parada do motor ========================== //

void pararMotorControle() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);

    // Zera estados do PID para evitar integral acumulada na próxima partida
    error = 0;
    error1 = 0;
    error2 = 0;

    output = 0;
    output1 = 0;
    output2 = 0;
}

// ================================================================== //
