#include "inputs.h"
#include <Arduino.h>

#define BOTAO_UP    13
#define BOTAO_DOWN  14
#define BOTAO_SET   27

#define POTENCIOMETRO 35

float setvel = 1000;   // valor em ajuste
float setpoint = 0;     // valor desejado

unsigned long ultimoTempo = 0;
unsigned long debounceDelay = 200;     // inicia em 200 ms
unsigned long TempoPressionado = 0;    // início do toque/botão

bool press = false;
int acrescimo = 0;

int pot_val = 0, ultimo_pot_val = 0, leitura_pot = 0;
float pot_tensao = 0;

void initInput() {
    pinMode(BOTAO_UP, INPUT_PULLUP);
    pinMode(BOTAO_DOWN, INPUT_PULLUP);
    pinMode(BOTAO_SET, INPUT_PULLUP);

    pinMode(POTENCIOMETRO, INPUT);
    analogSetPinAttenuation(POTENCIOMETRO, ADC_11db);
}

ValoresVelocidade EntradasVelocidade() {

    bool botaoUP   = digitalRead(BOTAO_UP)   == LOW;
    bool botaoDOWN = digitalRead(BOTAO_DOWN) == LOW;
    bool botaoSET  = digitalRead(BOTAO_SET)  == LOW;

    bool foiPressionado = botaoUP || botaoDOWN || botaoSET;

    // Detecta início do pressionamento
    if (foiPressionado && !press) {
        TempoPressionado = millis();
        debounceDelay = 200;   // começa lento
        acrescimo = 100;
        press = true;
    }

    // Se mantém pressionado por mais de 5 s, aumenta o incremento
    if (press && (millis() - TempoPressionado > 5000)) {
        debounceDelay = 300;
        acrescimo = 1000;
    }

    // Reseta se soltar todos os botões
    if (!foiPressionado) {
        press = false;
    }

    // Verifica debounce
    if (millis() - ultimoTempo > debounceDelay) {

        if (botaoUP) {
            setvel += acrescimo;

            if (setvel > 12000) {
                setvel = 12000;
            }

            ultimoTempo = millis();
        }
        else if (botaoDOWN) {
            setvel -= acrescimo;

            if (setvel < -12000) {
                setvel = -12000;
            }

            ultimoTempo = millis();
        }
        else if (botaoSET) {
            setpoint = setvel;
            ultimoTempo = millis();
        }
    }

    pot_val = analogRead(POTENCIOMETRO);

    /*
    if (abs(pot_val - ultimo_pot_val) > 100) {
        ultimo_pot_val = pot_val;

        pot_tensao = (float)pot_val / 4095.0f;
        pot_tensao = pot_tensao * 3.3f;

        setvel = (int)((pot_tensao - 1.65f) * (12000.0f / 1.65f));

        if (botaoSET) {
            setpoint = setvel;
            ultimoTempo = millis();
        }
    }
    */

    ValoresVelocidade valores;
    valores.setvel = setvel;
    valores.setpoint = setpoint;

    return valores;
}

void ResetEntradasVelocidade() {
    setvel = 0.0f;
    setpoint = 0.0f;

    ultimoTempo = millis();
    debounceDelay = 200;
    TempoPressionado = 0;

    press = false;
    acrescimo = 0;

    pot_val = 0;
    ultimo_pot_val = 0;
    leitura_pot = 0;
    pot_tensao = 0.0f;
}