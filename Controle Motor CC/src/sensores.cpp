#include "sensores.h"
#include <Arduino.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>


// Pinout
#define PIN_SENSOR_QUADRATURA_A 32
#define PIN_SENSOR_QUADRATURA_B 25
#define PIN_SENSOR_VELOCIDADE   33

// ========================  ADS1115  ======================== //

#define ADS1115_ADDR          0x48

#define ADS_CANAL_TENSAO      1     // A1 do ADS1115 falta ajustar tudo

#define ADS_GAIN_CORRENTE     GAIN_SIXTEEN
#define ADS_GAIN_TENSAO       GAIN_TWO

#define RSHUNT_OHMS           0.018f

// divisor 5.1k/(5.1k + 47k)
#define FATOR_DIVISOR_TENSAO        10.2157f

#define FILTRO_CORRENTE              0.70f
#define FILTRO_TENSAO                0.70f

Adafruit_ADS1115 ads;
bool ads_ok = false;

double corrente_filtrada = 0.0f;
double tensao_filtrada   = 0.0f;

bool primeira_corrente = true;
bool primeira_tensao   = true;

double lerTensaoADS(uint8_t canal) {
    if (!ads_ok) {
        return 0.0f;
    }

    int16_t leitura = ads.readADC_SingleEnded(canal);
    return ads.computeVolts(leitura);
}

// ============================================================ //

// ==========================  Encoder  ========================== //

#define N_FUROS               100        // número de furos do encoder

#define MIN_DEBOUNCE_US       10          // evita ruidos do sensor 
#define RPM_MAX               12000

/*
   RPM_FILT_ALPHA:
   Peso da leitura anterior no filtro da velocidade.

   Como RPM_FILT_ALPHA = 0.35:
   - 35% da leitura anterior;
   - 65% da leitura nova.

   Isso reduz degraus e ruídos na velocidade sem deixar a resposta
   excessivamente lenta para o PI.
*/
#define RPM_FILT_ALPHA        0.35f

/*
   Janelas de medição de velocidade.

   Mantidas em 10 ms para ficarem compatíveis com o tempo de amostragem
   do PI. Acima da faixa de baixa velocidade, a velocidade é calculada
   por contagem de pulsos nessa janela.
*/
#define VEL_WINDOW_MUITO_BAIXA_MS  9
#define VEL_WINDOW_BAIXA_MS        9
#define VEL_WINDOW_MEDIA_MS        9
#define VEL_WINDOW_ALTA_MS         9

/*
   Método híbrido de velocidade:

   - abaixo de RPM_HYBRID_SWITCH, usa período entre pulsos;
   - acima de RPM_HYBRID_SWITCH, usa contagem de pulsos na janela.

   Isso melhora a leitura em baixa velocidade, onde a janela de 10 ms
   gera degraus grandes, mas mantém uma leitura mais estável em alta
   velocidade, onde há muitos pulsos por janela.
*/
#define RPM_HYBRID_SWITCH     2000.0f

/*
   Decaimento quando não há pulsos e o último pulso já ficou antigo.

   Se não chegou pulso na janela atual, mas o último pulso ainda é recente,
   o código mantém a estimativa por período. Isso é importante em baixa
   rotação, pois pode não chegar pulso em toda janela de 10 ms.
*/
#define RPM_ZERO_DECAY        0.85f

/*
   Banda morta para zerar a velocidade residual.

   Quando não há pulsos e a velocidade filtrada já está muito pequena,
   é melhor zerar para evitar que o PI trabalhe com um valor residual.
*/
#define RPM_ZERO_BAND         2.0f

/*
   Timeout para considerar que a informação do último período ficou antiga.

   O timeout é calculado como múltiplo do último período medido, com limites
   mínimo e máximo. Assim, em baixa velocidade, o código não zera a velocidade
   só porque uma janela de 10 ms não recebeu pulso.
*/
#define RPM_PULSE_TIMEOUT_MULT     3UL
#define RPM_PULSE_TIMEOUT_MIN_US   30000UL
#define RPM_PULSE_TIMEOUT_MAX_US   300000UL

#define ENC_INVERT_DIR false             // ajuste de sentido (direção positiva e negattiva)

#define DIR_MIN_DIF_US       10

/*
   Limiar para aceitar mudança de sentido pela quadratura.

   Valor 0 aceita qualquer leitura válida da direção, pois agora o sentido
   é definido pela diferença entre o atraso da subida e o atraso da descida.
*/
#define MIN_DIR_SET           0

volatile int32_t pulseCount_modulo = 0;
volatile int32_t pulseCount_dir    = 0;

volatile uint8_t dir_a_prev = 0;
volatile uint8_t dir_b_prev = 0;

volatile uint32_t t_b_subiu = 0;
volatile uint32_t t_a_desceu = 0;

volatile uint32_t atraso_subida = 0;
volatile uint32_t atraso_descida = 0;

volatile bool atraso_subida_valido = false;
volatile bool atraso_descida_valido = false;

/*
   Variáveis para medição por período entre pulsos.

   Elas são atualizadas dentro da interrupção do sensor de velocidade.
   A função VelocidadeMotor() apenas copia esses valores e calcula a
   melhor estimativa de RPM a cada chamada.
*/
volatile uint32_t ultimoPulsoModulo_us  = 0;
volatile uint32_t periodoPulsoModulo_us = 0;
volatile bool periodoPulsoValido        = false;

int8_t sentidoAtual = 0;

/*
   Sentido escolhido a partir da velocidade selecionada pelo usuário.

   Esse valor será usado apenas quando o motor estiver parado ou praticamente
   parado, para evitar que uma leitura inicial errada da quadratura defina
   o sentido invertido.
*/
int8_t sentidoSelecionado = 0;

float RPM_ant = 0.0f;

unsigned long UltimaMedidaTempo = 0;

// ================================================================ //

// ========================= Interrupções ========================= //

void IRAM_ATTR contarPulsosModulo() {
    static uint32_t last = 0;
    uint32_t now = micros();

    if ((now - last) < MIN_DEBOUNCE_US) return;

    /*
       Além de contar o pulso, mede também o período entre o pulso atual
       e o pulso anterior.

       Esse período será usado em baixa velocidade, onde a contagem em
       janela de 10 ms fica muito quantizada.
    */
    if (ultimoPulsoModulo_us != 0) {
        periodoPulsoModulo_us = now - ultimoPulsoModulo_us;
        periodoPulsoValido = true;
    }

    ultimoPulsoModulo_us = now;
    last = now;

    pulseCount_modulo++;
}

void IRAM_ATTR contarEncoderDir() {
    static uint32_t last = 0;
    uint32_t now = micros();

    if ((now - last) < MIN_DEBOUNCE_US) return;

    last = now;

    uint8_t a = gpio_get_level((gpio_num_t)PIN_SENSOR_QUADRATURA_A);
    uint8_t b = gpio_get_level((gpio_num_t)PIN_SENSOR_QUADRATURA_B);

    bool a_subiu  = (!dir_a_prev && a);
    bool a_desceu = ( dir_a_prev && !a);
    bool b_subiu  = (!dir_b_prev && b);
    bool b_desceu = ( dir_b_prev && !b);

    if (b_subiu) {
        t_b_subiu = now;
    }

    if (a_subiu && b && t_b_subiu != 0) {
        atraso_subida = now - t_b_subiu;
        atraso_subida_valido = true;
    }

    if (a_desceu) {
        t_a_desceu = now;
    }

    if (b_desceu && !a && t_a_desceu != 0) {
        atraso_descida = now - t_a_desceu;
        atraso_descida_valido = true;
    }

    if (atraso_subida_valido && atraso_descida_valido) {
        int8_t delta = 0;

        if (atraso_subida > (atraso_descida + DIR_MIN_DIF_US)) {
            delta = 1;
        } else if (atraso_descida > (atraso_subida + DIR_MIN_DIF_US)) {
            delta = -1;
        }

        if (ENC_INVERT_DIR) delta = -delta;

        pulseCount_dir += delta;

        atraso_subida_valido = false;
        atraso_descida_valido = false;
    }

    dir_a_prev = a;
    dir_b_prev = b;
}

// ================================================================ //

void initSensores() {
    pinMode(PIN_SENSOR_QUADRATURA_A, INPUT);
    pinMode(PIN_SENSOR_QUADRATURA_B, INPUT);
    pinMode(PIN_SENSOR_VELOCIDADE,   INPUT);

    ads.setGain(ADS_GAIN_CORRENTE);
    ads_ok = ads.begin(ADS1115_ADDR);

    if (!ads_ok) {
        Serial.println("ADS1115 nao encontrado no endereco 0x48");
    }

    // quadratura (direção)
    dir_a_prev = gpio_get_level((gpio_num_t)PIN_SENSOR_QUADRATURA_A);
    dir_b_prev = gpio_get_level((gpio_num_t)PIN_SENSOR_QUADRATURA_B);
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_QUADRATURA_A), contarEncoderDir, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_QUADRATURA_B), contarEncoderDir, CHANGE);

    // módulo (velocidade)
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_VELOCIDADE), contarPulsosModulo, FALLING);


    UltimaMedidaTempo = millis();
}

void deinitSensores() {
}

/*
   VelocidadeMotor()

   Mantém a lógica:

   1) PIN_SENSOR_VELOCIDADE mede o módulo da velocidade;
   2) PIN_SENSOR_QUADRATURA_A e PIN_SENSOR_QUADRATURA_B determinam o sentido;
   3) velocidade final:
          rpm_raw = sentidoAtual * rpm_abs;

   Agora o módulo da velocidade é calculado de forma híbrida:

   - em baixa velocidade, usa período entre pulsos;
   - em média/alta velocidade, usa contagem de pulsos em janela de 10 ms.

   Isso melhora a leitura em baixa rotação sem mudar a estrutura do PI.
*/

void SetSentidoSelecionado(float velocidadeSelecionada) {
    if (velocidadeSelecionada > 0.0f) {
        sentidoSelecionado = 1;
    } else if (velocidadeSelecionada < 0.0f) {
        sentidoSelecionado = -1;
    } else {
        sentidoSelecionado = 0;
    }
}

float VelocidadeMotor() {
    unsigned long agora = millis();
    unsigned long TempoDecorrido_ms = agora - UltimaMedidaTempo;

    unsigned long min_window;

    // Janela fixa de 10 ms, compatível com o PI.
    float RPM_ant_abs = fabsf(RPM_ant);

    /*if      (RPM_ant_abs < 200.0f)  min_window = VEL_WINDOW_MUITO_BAIXA_MS;
    else if (RPM_ant_abs < 1000.0f) min_window = VEL_WINDOW_BAIXA_MS;
    else if (RPM_ant_abs < 5000.0f) min_window = VEL_WINDOW_MEDIA_MS;
    else                            min_window = VEL_WINDOW_ALTA_MS;

    // Enquanto a janela não fecha, retorna a última velocidade calculada.
    if (TempoDecorrido_ms < min_window) {
        return RPM_ant;
    }*/

    int32_t pulses_mod;
    int32_t pulses_dir;

    uint32_t periodo_us;
    uint32_t ultimoPulso_us;
    bool periodo_valido;

    noInterrupts();

    pulses_mod = pulseCount_modulo;
    pulseCount_modulo = 0;

    pulses_dir = pulseCount_dir;
    pulseCount_dir = 0;

    /*
       Copia os dados de período medidos na interrupção.

       Eles não são zerados aqui, pois o último período continua sendo útil
       quando a rotação é baixa e não chega pulso em toda janela de 10 ms.
    */
    periodo_us = periodoPulsoModulo_us;
    ultimoPulso_us = ultimoPulsoModulo_us;
    periodo_valido = periodoPulsoValido;

    interrupts();

    UltimaMedidaTempo = agora;

    /*
   Se o motor está parado ou praticamente parado, o sentido não deve ser
   definido pela quadratura, pois qualquer ruído inicial pode inverter
   sentidoAtual.

    Nesse caso, usa o sentido escolhido pela velocidade selecionada.
    */
    if (fabsf(RPM_ant) < RPM_ZERO_BAND) {
        sentidoAtual = sentidoSelecionado;
    }

    /*
    Atualização do sentido pela quadratura.

    A quadratura só deve corrigir o sentido quando:
    - houve pulso no sensor de velocidade;
    - o motor não está praticamente parado.

    Isso evita que, na partida, uma leitura falsa coloque sentidoAtual
    invertido.
    */

    //Serial.print(pulses_dir);
    //Serial.println();

    if (pulses_mod > 0 && fabsf(RPM_ant) >= RPM_ZERO_BAND) {
        if (pulses_dir > MIN_DIR_SET) {
            sentidoAtual = 1;
        } else if (pulses_dir < -MIN_DIR_SET) {
            sentidoAtual = -1;
        }
    }
    
    //Serial.print(sentidoAtual);
    //Serial.println();
    /*
       Cálculo da velocidade pelo período entre pulsos.

       Fórmula:
          RPM = 60*10^6 / (periodo_us * N_FUROS)

       Essa estimativa é melhor em baixa velocidade.
    */
    float rpm_abs_periodo = 0.0f;

    if (periodo_valido && periodo_us > 0) {
        rpm_abs_periodo = (60.0f * 1000000.0f) / ((float)periodo_us * (float)N_FUROS);
        rpm_abs_periodo = constrain(rpm_abs_periodo, 0.0f, (float)RPM_MAX);
    }

    /*
       Quando não há pulso na janela de 10 ms:

       - se o último pulso ainda é recente, usa a estimativa por período;
       - se o último pulso ficou antigo, aplica decaimento até zerar.

       Isso evita que a leitura fique caindo artificialmente em baixa rotação,
       onde é normal não receber pulso em toda janela de 10 ms.
    */
    if (pulses_mod == 0) {
        bool pulso_recente = false;

        if (periodo_valido && periodo_us > 0 && ultimoPulso_us != 0) {
            uint32_t agora_us = micros();
            uint32_t idadePulso_us = agora_us - ultimoPulso_us;

            uint32_t timeout_us;

            if (periodo_us > (RPM_PULSE_TIMEOUT_MAX_US / RPM_PULSE_TIMEOUT_MULT)) {
                timeout_us = RPM_PULSE_TIMEOUT_MAX_US;
            } else {
                timeout_us = periodo_us * RPM_PULSE_TIMEOUT_MULT;
            }

            if (timeout_us < RPM_PULSE_TIMEOUT_MIN_US) {
                timeout_us = RPM_PULSE_TIMEOUT_MIN_US;
            }

            if (timeout_us > RPM_PULSE_TIMEOUT_MAX_US) {
                timeout_us = RPM_PULSE_TIMEOUT_MAX_US;
            }

            if (idadePulso_us <= timeout_us) {
                pulso_recente = true;
            }
        }


        if (pulso_recente && rpm_abs_periodo > 0.0f && rpm_abs_periodo < RPM_HYBRID_SWITCH) {
            float rpm_raw = sentidoAtual * rpm_abs_periodo;

            rpm_raw = constrain(rpm_raw, -(float)RPM_MAX, (float)RPM_MAX);

            RPM_ant = RPM_FILT_ALPHA * RPM_ant + (1.0f - RPM_FILT_ALPHA) * rpm_raw;

            if (fabsf(RPM_ant) < RPM_ZERO_BAND) {
                RPM_ant = 0.0f;
                sentidoAtual = sentidoSelecionado;
            }


            return RPM_ant;
        }

        /*
           Se o pulso não é recente, considera que a velocidade está caindo
           ou que o motor parou.
        */
        RPM_ant *= RPM_ZERO_DECAY;

        if (fabsf(RPM_ant) < RPM_ZERO_BAND) {
            RPM_ant = 0.0f;
            sentidoAtual = sentidoSelecionado;
        }

        return RPM_ant;
    }

    /*
       Cálculo da velocidade por contagem de pulsos na janela.

       Essa estimativa é melhor em média/alta velocidade, pois há mais pulsos
       dentro da janela de 10 ms.
    */
    float time_s = TempoDecorrido_ms / 1000.0f;

    float rpm_abs_janela = (pulses_mod * 60.0f) / (time_s * N_FUROS);
    rpm_abs_janela = constrain(rpm_abs_janela, 0.0f, (float)RPM_MAX);

    /*
       Seleção híbrida:

       - se a estimativa por período estiver abaixo de 2000 RPM, usa período;
       - caso contrário, usa contagem por janela.
    */
    float rpm_abs;

    if (periodo_valido && rpm_abs_periodo > 0.0f && rpm_abs_periodo < RPM_HYBRID_SWITCH) {
        rpm_abs = rpm_abs_periodo;
    } else {
        rpm_abs = rpm_abs_janela;
    }

    /*
       Mantém exatamente a lógica desejada:

       - rpm_abs vem do sensor de velocidade;
       - sentidoAtual vem da quadratura;
       - velocidade final depende do sinal de sentidoAtual.
    */
    float rpm_raw = sentidoAtual * rpm_abs;

    // Limita também a velocidade com sinal.
    rpm_raw = constrain(rpm_raw, -(float)RPM_MAX, (float)RPM_MAX);

    /*
       Filtro leve para reduzir degraus de medição e ruído.
    */
    RPM_ant = RPM_FILT_ALPHA * RPM_ant + (1.0f - RPM_FILT_ALPHA) * rpm_raw;

    // Evita que valores residuais muito pequenos fiquem alimentando o PI.
    if (fabsf(RPM_ant) < RPM_ZERO_BAND) {
        RPM_ant = 0.0f;
    }

    return RPM_ant;
}

float CorrenteMotor() {

    if (!ads_ok) {
        return 0.0f;
    }

    ads.setGain(ADS_GAIN_CORRENTE);

    int16_t leitura = ads.readADC_Differential_0_3();

    double vshunt = ads.computeVolts(leitura);
    vshunt = trunc(vshunt * 1000.0) / 1000.0;

    double corrente = ((fabs(vshunt - 0.001f)) / RSHUNT_OHMS)*1.262;//1.1852;
    //double corrente = fabs(vshunt - 0.001f); //Ajuste devido corrente mesmo com motor parado

    if (primeira_corrente) {
        corrente_filtrada = corrente;
        primeira_corrente = false;
    } else {
        corrente_filtrada = FILTRO_CORRENTE * corrente_filtrada +
                            (1.0f - FILTRO_CORRENTE) * corrente;
    }

    return corrente_filtrada;
}

float TensaoMotor() {

    if (!ads_ok) {
        return 0.0f;
    }

    ads.setGain(ADS_GAIN_TENSAO);

    float tensao_ads = lerTensaoADS(ADS_CANAL_TENSAO);

    float tensao = fabs(tensao_ads * FATOR_DIVISOR_TENSAO);

    if (primeira_tensao) {
        tensao_filtrada = tensao;
        primeira_tensao = false;
    } else {
        tensao_filtrada = FILTRO_TENSAO * tensao_filtrada +
                          (1.0f - FILTRO_TENSAO) * tensao;
    }

    return tensao_filtrada;
}