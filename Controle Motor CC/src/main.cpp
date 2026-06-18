#include <Arduino.h>
#include "display.h"
#include "controleMotorCC.h"
#include "sensores.h"
#include "inputs.h"

#define PIN_DETECT_MODULO 17

float g_velocidade = 0.0f;
float g_corrente   = 0.0f;
float g_tensao     = 0.0f;
float g_spdisplay  = 0.0f;

const unsigned long t_atualizacao = 10;  // do PID em ms
unsigned long ultimo_t = 0;

bool modulo_conectado_anterior = false;

bool moduloConectado() {
  return digitalRead(PIN_DETECT_MODULO) == LOW;
}

// Display roda em paralelo em um nucleo separado
void displayTask(void *pvParameters) {
  for (;;) {

    unsigned long tempo_display_ms = 100;

    if (!moduloConectado()) {
      g_corrente   = 0.0f;
      g_tensao     = 0.0f;
      g_velocidade = 0.0f;
      g_spdisplay  = 0.0f;

      updateDisplay(0.0f, 0.0f, 0.0f, 0.0f);
    } 
    
    else {
      g_corrente = CorrenteMotor();
      g_tensao   = TensaoMotor();

      // usa sempre os últimos valores globais
      updateDisplay(g_velocidade, g_corrente, g_tensao, g_spdisplay);
    }

    vTaskDelay(pdMS_TO_TICKS(tempo_display_ms)); // atualiza a cada 100 ms
  }
}


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

void loop() {
  unsigned long agora = millis();

  if ((long)(agora - ultimo_t) >= (long)t_atualizacao) {
    ultimo_t += t_atualizacao;

    bool modulo_atual = moduloConectado();

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
    
    ValoresVelocidade valores = EntradasVelocidade();
    /*
    Informa ao código dos sensores qual é o sentido esperado na partida.

    Se valores.setvel > 0, sentido inicial positivo.
    Se valores.setvel < 0, sentido inicial negativo.
    Se valores.setvel = 0, sentido indefinido.
    */
    SetSentidoSelecionado(valores.setvel);

    g_velocidade = VelocidadeMotor();

    //g_corrente   = CorrenteMotor();
    //g_tensao     = TensaoMotor();

    g_spdisplay = valores.setvel;

    if (valores.setpoint == 0.0f && fabs(g_velocidade) < 2.0f) {
      pararMotorControle();
    } else {
      controleMotor(g_velocidade, valores.setpoint);
    }
  }
}