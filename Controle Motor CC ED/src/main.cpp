#include <Arduino.h>
#include "display.h"
#include "controleMotorCC.h"
#include "sensores.h"
#include "inputs.h"

// ===================== Definições de pinos ====================== //

#define PIN_DETECT_MODULO 17

// ================================================================== //

// ===================== Constantes =============================== //

constexpr unsigned long T_ATUALIZACAO_MS = 3;   // período do PI em ms
constexpr unsigned long T_DISPLAY_MS     = 100;  // período do OLED em ms
constexpr float VELOCIDADE_PARADA_RPM    = 2.0f; // velocidade de parada em RPM

// ================================================================== //

// ===================== Variáveis globais ======================== //

// Valores compartilhados entre o loop principal (core 1) e a task do display (core 0)
float g_velocidade = 0.0f;
float g_corrente   = 0.0f;
float g_tensao     = 0.0f;
float g_spdisplay  = 0.0f;

unsigned long ultimo_t = 0;

// Detecta transição de módulo desconectado → conectado para reset seguro
bool modulo_conectado_anterior = false;

// ================================================================== //

// ===================== Detecção do módulo ======================= //

bool moduloConectado() {
    return digitalRead(PIN_DETECT_MODULO) == LOW;
}

// ================================================================== //

// ===================== Task do display (core 0) ================= //

// Display roda em paralelo em um nucleo separado
void displayTask(void *pvParameters) {
    for (;;) {

        if (!moduloConectado()) {
            g_corrente   = 0.0f;
            g_tensao     = 0.0f;
            g_velocidade = 0.0f;
            g_spdisplay  = 0.0f;

            updateDisplay(0.0f, 0.0f, 0.0f, 0.0f);
        } else {
            g_corrente = CorrenteMotor();
            g_tensao   = TensaoMotor();

            // usa sempre os últimos valores globais
            updateDisplay(g_velocidade, g_corrente, g_tensao, g_spdisplay);
        }

        vTaskDelay(pdMS_TO_TICKS(T_DISPLAY_MS)); // atualiza a cada 100 ms
    }
}

// ================================================================== //

// ===================== Setup ====================================== //

void setup() {
    Serial.begin(9600);

    pinMode(PIN_DETECT_MODULO, INPUT_PULLUP);

    initDisplay();
    initMotorControle();
    initSensores();
    initInput();
    showMessage("Sistema Iniciado");

    modulo_conectado_anterior = moduloConectado();

    // Atualização do display no core 0 (loop roda no core 1)
    xTaskCreatePinnedToCore(
        displayTask,      // função
        "DisplayTask",    // nome
        4096,             // stack
        NULL,             // pvParameters
        1,                // prioridade
        NULL,             // handle (opcional)
        0                 // core 0
    );
}

// ================================================================== //

// ===================== Loop principal (core 1) ==================== //

void loop() {
    unsigned long agora = millis();

    if ((long)(agora - ultimo_t) >= (long)T_ATUALIZACAO_MS) {
        ultimo_t += T_ATUALIZACAO_MS;

        bool modulo_atual = moduloConectado();

        // ===================== Módulo desconectado ================== //

        if (!modulo_atual) {
            modulo_conectado_anterior = false;

            g_velocidade = 0.0f;
            g_corrente   = 0.0f;
            g_tensao     = 0.0f;
            g_spdisplay  = 0.0f;

            SetSentidoSelecionado(0.0f);

            ResetEntradasVelocidade();
            pararMotorControle();

            return;
        }

        // ===================== Módulo reconectado ================= //

        if (modulo_atual && !modulo_conectado_anterior) {
            modulo_conectado_anterior = true;

            g_velocidade = 0.0f;
            g_corrente   = 0.0f;
            g_tensao     = 0.0f;
            g_spdisplay  = 0.0f;

            SetSentidoSelecionado(0.0f);

            ResetEntradasVelocidade();
            pararMotorControle();

            return;
        }

        // ===================== Leitura e controle =================== //

        ValoresVelocidade valores = EntradasVelocidade();
        /*
        Informa ao código dos sensores qual é o sentido esperado na partida.

        Se valores.setvel > 0, sentido inicial positivo.
        Se valores.setvel < 0, sentido inicial negativo.
        Se valores.setvel = 0, sentido indefinido.
        */
        SetSentidoSelecionado(valores.setvel);

        g_velocidade = VelocidadeMotor();

        g_spdisplay = valores.setvel;

        // Para o motor quando setpoint = 0 e velocidade já está próxima de zero
        if (valores.setpoint == 0.0f && fabs(g_velocidade) < VELOCIDADE_PARADA_RPM) {
            pararMotorControle();
        } else {
            controleMotor(g_velocidade, valores.setpoint);
        }
    }
}

// ================================================================== //
