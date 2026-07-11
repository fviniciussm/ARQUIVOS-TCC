#include "inputs.h"
#include <Arduino.h>

// ===================== Definições de pinos ====================== //

#define BOTAO_UP    13
#define BOTAO_DOWN  14
#define BOTAO_SET   27

// ================================================================== //

// ===================== Constantes =============================== //

constexpr float RPM_LIMITE                    = 12000.0f; // valor máximo de velocidade
constexpr unsigned long DEBOUNCE_INICIAL_MS   = 200; // tempo de debounce inicial
constexpr unsigned long DEBOUNCE_RAPIDO_MS    = 300; // tempo de debounce rápido
constexpr unsigned long TEMPO_ACELERACAO_MS   = 5000; // tempo de aceleração
constexpr int ACRESCIMO_INICIAL               = 100; // valor de incremento inicial
constexpr int ACRESCIMO_RAPIDO                = 1000; // valor de incremento rápido

// ================================================================== //

// ===================== Variáveis ================================ //

float setvel = 1000;   // valor em ajuste
float setpoint = 0;     // valor desejado

unsigned long ultimoTempo = 0;
unsigned long debounceDelay = DEBOUNCE_INICIAL_MS;
unsigned long TempoPressionado = 0;

bool press = false;
int acrescimo = 0;

// ================================================================== //

// ===================== Inicialização ============================ //

void initInput() {
    pinMode(BOTAO_UP, INPUT_PULLUP);
    pinMode(BOTAO_DOWN, INPUT_PULLUP);
    pinMode(BOTAO_SET, INPUT_PULLUP);
}

// ================================================================== //

// ===================== Leitura das entradas ===================== //

ValoresVelocidade EntradasVelocidade() {

    bool botaoUP   = digitalRead(BOTAO_UP)   == LOW;
    bool botaoDOWN = digitalRead(BOTAO_DOWN) == LOW;
    bool botaoSET  = digitalRead(BOTAO_SET)  == LOW;

    bool foiPressionado = botaoUP || botaoDOWN || botaoSET;

    // Detecta início do pressionamento
    if (foiPressionado && !press) {
        TempoPressionado = millis();
        debounceDelay = DEBOUNCE_INICIAL_MS;
        acrescimo = ACRESCIMO_INICIAL;
        press = true;
    }

    // Se mantém pressionado por mais de 5 s, aumenta o incremento
    if (press && (millis() - TempoPressionado > TEMPO_ACELERACAO_MS)) {
        debounceDelay = DEBOUNCE_RAPIDO_MS;
        acrescimo = ACRESCIMO_RAPIDO;
    }

    // Reseta se soltar todos os botões
    if (!foiPressionado) {
        press = false;
    }

    // Verifica debounce
    if (millis() - ultimoTempo > debounceDelay) {

        if (botaoUP) {
            setvel += acrescimo;

            if (setvel > RPM_LIMITE) {
                setvel = RPM_LIMITE;
            }

            ultimoTempo = millis();
        }
        else if (botaoDOWN) {
            setvel -= acrescimo;

            if (setvel < -RPM_LIMITE) {
                setvel = -RPM_LIMITE;
            }

            ultimoTempo = millis();
        }
        else if (botaoSET) {
            setpoint = setvel;
            ultimoTempo = millis();
        }
    }

    ValoresVelocidade valores;
    valores.setvel = setvel;
    valores.setpoint = setpoint;

    return valores;
}

// ================================================================== //

// ===================== Reset das entradas ========================= //

void ResetEntradasVelocidade() {
    setvel = 0.0f;
    setpoint = 0.0f;

    ultimoTempo = millis();
    debounceDelay = DEBOUNCE_INICIAL_MS;
    TempoPressionado = 0;

    press = false;
    acrescimo = 0;
}

// ================================================================== //
