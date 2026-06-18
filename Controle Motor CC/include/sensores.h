#ifndef SENSORES_H
#define SENSORES_H

void initSensores();

float VelocidadeMotor();     // RPM
float CorrenteMotor();       // A
float TensaoMotor();         // V

void SetSentidoSelecionado(float velocidadeSelecionada);

#endif