#ifndef MOTOR_CONTROLE_H
#define MOTOR_CONTROLE_H

// ===================== Controle do motor CC ===================== //

void initMotorControle();
void controleMotor(float velocidade, float setpoint);
void pararMotorControle();

#endif
