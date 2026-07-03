#include "sensores.h"
#include <Arduino.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>

// ===================== Definições de pinos ====================== //

#define PIN_SENSOR_QUADRATURA_A 32
#define PIN_SENSOR_QUADRATURA_B 25
#define PIN_SENSOR_VELOCIDADE   33

// ================================================================== //

// ===================== Constantes =============================== //

// ADS1115

constexpr uint8_t ADS1115_ADDR     = 0x48;
constexpr uint8_t ADS_CANAL_TENSAO = 1;  // A1 do ADS1115

constexpr auto ADS_GAIN_CORRENTE = GAIN_SIXTEEN;
constexpr auto ADS_GAIN_TENSAO   = GAIN_TWO;

constexpr float RSHUNT_OHMS = 0.018f;

// divisor 5.1k/(5.1k + 47k)
constexpr float FATOR_DIVISOR_TENSAO = 10.2157f;

constexpr float FILTRO_CORRENTE = 0.70f;
constexpr float FILTRO_TENSAO   = 0.70f;

// Encoder
constexpr int N_FUROS              = 100;  // número de furos do encoder
constexpr uint32_t MIN_DEBOUNCE    = 10;   // debounce das ISRs (µs)
constexpr int RPM_MAX              = 12000; // valor máximo de velocidade

/*
   RPM_FILT:
   Peso da leitura anterior no filtro da velocidade.

   Como RPM_FILT = 0.35:
   - 35% da leitura anterior;
   - 65% da leitura nova.

   Reduz degraus e ruídos na velocidade sem deixar a resposta
   excessivamente lenta para o PI.
*/

constexpr float RPM_FILT = 0.35f;

/*
   Método de leitura de velocidade:

   - abaixo de LIMIAR_LEITURA_RPM, usa leitura por período entre pulsos;
   - acima de LIMIAR_LEITURA_RPM, usa leitura por contagem de pulsos na janela.

   Isso melhora a leitura em baixa velocidade, onde a janela de 10 ms
   gera degraus grandes, e em alta velocidade, onde há muitos pulsos por janela.
*/
constexpr float LIMIAR_LEITURA_RPM = 2000.0f; // velocidade de transição entre tipo de leitura

/*
   RPM_DECAIMENTO:
   Fator aplicado a RPM_ant quando nenhum pulso chega na janela e o último
   pulso já passou do timeout (não é mais considerado recente).

   A cada ciclo de 10 ms nessa condição, a velocidade cai 15% (× 0,85) até
   ficar abaixo de RPM_BANDA_MORTA e ser zerada.
*/
constexpr float RPM_DECAIMENTO = 0.85f;

/*
   Banda morta para zerar a velocidade residual:

   Quando não há pulsos e a velocidade filtrada já está muito pequena,
   é melhor zerar para evitar que o PI trabalhe com um valor residual.
*/
constexpr float RPM_BANDA_MORTA = 2.0f;

/*
   Timeout do último pulso (valores em microssegundos).

   Em baixa rotação é normal que uma janela de 10 ms não receba pulso.
   O código não pode concluir "motor parou" só por isso.

   Passo a passo em VelocidadeMotor(), quando pulses_mod == 0:
   1) Mede idadePulso = tempo desde ultimoPulsoModulo.
   2) Estima timeout = periodoPulsoModulo * RPM_PULSE_TIMEOUT_MULT.
   3) Limita timeout entre RPM_PULSE_TIMEOUT_MIN (30 ms) e
      RPM_PULSE_TIMEOUT_MAX (300 ms).
   4) Se idadePulso <= timeout, o pulso ainda é recente: mantém a RPM
      calculada pelo período entre pulsos.
   5) Se idadePulso > timeout, aplica RPM_DECAIMENTO até zerar.

   Exemplos com RPM_PULSE_TIMEOUT_MULT = 3:
   - periodo = 50 ms -> timeout = 150 ms
   - periodo =  5 ms -> timeout = 15 ms, sobe para MIN = 30 ms
   - periodo = 200 ms -> timeout = 600 ms, limita em MAX = 300 ms
*/
constexpr uint32_t RPM_PULSE_TIMEOUT_MULT = 3UL;
constexpr uint32_t RPM_PULSE_TIMEOUT_MIN  = 30000UL;   // 30 ms
constexpr uint32_t RPM_PULSE_TIMEOUT_MAX  = 300000UL;  // 300 ms

constexpr bool ENC_INVERT_DIR = false;  // ajuste de sentido (direção positiva e negativa)

constexpr uint32_t DIR_MIN_DIF = 10;  // diferença mínima entre atrasos da quadratura (µs)

/*
   Limiar para aceitar mudança de sentido pela quadratura.

   Valor 0 aceita qualquer leitura válida da direção, pode ser necessário mudar.
*/
constexpr int MIN_DIR_SET = 0;

// ================================================================== //

// ===================== Variáveis ================================ //

Adafruit_ADS1115 ads;
bool ads_ok = false;

double corrente_filtrada = 0.0f;
double tensao_filtrada   = 0.0f;

bool primeira_corrente = true;
bool primeira_tensao   = true;

// ================================================================== //

// ===================== Leitura auxiliar ADS ===================== //

double lerTensaoADS(uint8_t canal) {
    if (!ads_ok) {
        return 0.0f;
    }

    int16_t leitura = ads.readADC_SingleEnded(canal);
    return ads.computeVolts(leitura);
}

// ================================================================== //

// ===================== Variáveis do encoder ===================== //

// Contadores de pulsos (atualizados nas ISRs)
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
   Período entre pulsos e instante do último pulso (µs).

   Atualizados na ISR contarPulsosModulo(). VelocidadeMotor() copia esses
   valores para estimar RPM por período e decidir se o último pulso ainda
   é recente (ver RPM_PULSE_TIMEOUT_*).
*/
volatile uint32_t ultimoPulsoModulo  = 0;
volatile uint32_t periodoPulsoModulo = 0;
volatile bool periodoPulsoValido        = false;

int8_t sentidoAtual = 0;

/*
   Sentido escolhido a partir da velocidade selecionada pelo usuário:

   Esse valor será usado apenas quando o motor estiver parado para evitar
   que uma leitura inicial errada da quadratura defina o sentido invertido.
*/
int8_t sentidoSelecionado = 0;

float RPM_ant = 0.0f;

unsigned long UltimaMedidaTempo = 0;

// ================================================================== //

// ========================= Interrupções ========================= //

void IRAM_ATTR contarPulsosModulo() {
    static uint32_t last = 0;
    uint32_t agora = micros();

    if ((agora - last) < MIN_DEBOUNCE) return;

    /*
       Além de contar o pulso, mede também o período entre o pulso atual
       e o pulso anterior.

       Esse período será usado em baixa velocidade, onde a contagem em
       janela de 10 ms fica muito quantizada.
    */
    if (ultimoPulsoModulo != 0) {
        periodoPulsoModulo = agora - ultimoPulsoModulo;
        periodoPulsoValido = true;
    }

    ultimoPulsoModulo = agora;
    last = agora;

    pulseCount_modulo++;
}

void IRAM_ATTR contarEncoderDir() {
    static uint32_t last = 0;
    uint32_t agora = micros();

    // Ignora transições muito próximas (ruído/elétrica).
    if ((agora - last) < MIN_DEBOUNCE) return;

    last = agora;

    uint8_t a = gpio_get_level((gpio_num_t)PIN_SENSOR_QUADRATURA_A);
    uint8_t b = gpio_get_level((gpio_num_t)PIN_SENSOR_QUADRATURA_B);

    // Borda detectada comparando o nível atual com o estado anterior (dir_x_prev).
    bool a_subiu  = (!dir_a_prev && a);
    bool a_desceu = ( dir_a_prev && !a);
    bool b_subiu  = (!dir_b_prev && b);
    bool b_desceu = ( dir_b_prev && !b);

    // Marca o instante em que B sobe; serve de referência para medir atraso_subida.
    if (b_subiu) {
        t_b_subiu = agora;
    }

    /*
       Sequência para medir atraso_subida:
       B sobe -> depois A sobe (com B ainda em nível alto).
       atraso_subida = tempo entre a subida de B e a subida de A.
    */
    if (a_subiu && b && t_b_subiu != 0) {
        atraso_subida = agora - t_b_subiu;
        atraso_subida_valido = true;
    }

    // Marca o instante em que A desce; referência para medir atraso_descida.
    if (a_desceu) {
        t_a_desceu = agora;
    }

    /*
       Sequência para medir atraso_descida:
       A desce -> depois B desce (com A ainda em nível baixo).
       atraso_descida = tempo entre a descida de A e a descida de B.
    */
    if (b_desceu && !a && t_a_desceu != 0) {
        atraso_descida = agora - t_a_desceu;
        atraso_descida_valido = true;
    }

    /*
       Só decide o sentido quando as duas sequências completas foram medidas.
       Compara os atrasos: quem chegou "atrasado" indica o sentido de rotação.
       DIR_MIN_DIF evita troca de sentido por diferença mínima/ruído.
    */
    if (atraso_subida_valido && atraso_descida_valido) {
        int8_t delta = 0;

        if (atraso_subida > (atraso_descida + DIR_MIN_DIF)) {
            delta = 1;
        } else if (atraso_descida > (atraso_subida + DIR_MIN_DIF)) {
            delta = -1;
        }

        // Inverte o sentido detectado se a fiação/quadratura estiver invertida.
        if (ENC_INVERT_DIR) delta = -delta;

        pulseCount_dir += delta;

        // Aguarda novo par de medições antes de decidir de novo.
        atraso_subida_valido = false;
        atraso_descida_valido = false;
    }

    // Guarda o estado atual para detectar bordas na próxima interrupção.
    dir_a_prev = a;
    dir_b_prev = b;
}

// ================================================================== //

// ===================== Inicialização ============================ //

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

// ================================================================== //

// ===================== Sentido selecionado ======================= //

void SetSentidoSelecionado(float velocidadeSelecionada) {
    if (velocidadeSelecionada > 0.0f) {
        sentidoSelecionado = 1;
    } else if (velocidadeSelecionada < 0.0f) {
        sentidoSelecionado = -1;
    } else {
        sentidoSelecionado = 0;
    }
}

// ================================================================== //

// ===================== Velocidade do motor ====================== //

/*
   VelocidadeMotor()
   Lógica:

   1) PIN_SENSOR_VELOCIDADE mede o módulo da velocidade;
   2) PIN_SENSOR_QUADRATURA_A e PIN_SENSOR_QUADRATURA_B determinam o sentido;
   3) velocidade final:
          rpm_raw = sentidoAtual * rpm_abs;

   O módulo da velocidade é calculado de forma híbrida:

   - em baixa velocidade, usa período entre pulsos;
   - em média/alta velocidade, usa contagem de pulsos em janela de 10 ms.

   Isso melhora a leitura em baixa rotação sem mudar a estrutura do PI.
*/

float VelocidadeMotor() {
    unsigned long agora = millis();
    unsigned long TempoDecorrido_ms = agora - UltimaMedidaTempo;

    // ===================== Leitura atômica dos contadores ========== //

    int32_t pulses_mod;
    int32_t pulses_dir;

    uint32_t periodo;
    uint32_t ultimoPulso;
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
    periodo = periodoPulsoModulo;
    ultimoPulso = ultimoPulsoModulo;
    periodo_valido = periodoPulsoValido;

    interrupts();

    UltimaMedidaTempo = agora;

    // ===================== Determinação do sentido ================ //

    /*
       Se o motor está parado o sentido não deve ser definido pela quadratura,
       pois qualquer ruído inicial pode inverter o sentidoAtual.

       Nesse caso, usa o sentido escolhido pela velocidade selecionada.
    */
    if (fabsf(RPM_ant) < RPM_BANDA_MORTA) {
        sentidoAtual = sentidoSelecionado;
    }

    /*
       Atualização do sentido pela quadratura:

       A quadratura só deve corrigir o sentido quando:
       - houve pulso no sensor de velocidade;
       - o motor não está praticamente parado.
    */

    if (pulses_mod > 0 && fabsf(RPM_ant) >= RPM_BANDA_MORTA) {
        if (pulses_dir > MIN_DIR_SET) {
            sentidoAtual = 1;
        } else if (pulses_dir < -MIN_DIR_SET) {
            sentidoAtual = -1;
        }
    }

    // ===================== Estimativa por período ================= //

    /*
       Cálculo da velocidade pelo período entre pulsos.

       Fórmula:
          RPM = 60*10^6 / (periodo * N_FUROS)
    */
    float rpm_abs_periodo = 0.0f;

    if (periodo_valido && periodo > 0) {
        rpm_abs_periodo = (60.0f * 1000000.0f) / ((float)periodo * (float)N_FUROS);
        rpm_abs_periodo = constrain(rpm_abs_periodo, 0.0f, (float)RPM_MAX);
    }

    // ===================== Sem pulsos na janela =================== //

    /*
       Janela de 10 ms sem pulso:

       - calcula idadePulso e timeout (RPM_PULSE_TIMEOUT_*);
       - se o pulso ainda é recente, mantém RPM por período;
       - se expirou, aplica RPM_DECAIMENTO até zerar.
    */
    if (pulses_mod == 0) {
        bool pulso_recente = false;

        if (periodo_valido && periodo > 0 && ultimoPulso != 0) {
            uint32_t instante = micros();
            uint32_t idadePulso = instante - ultimoPulso;

            // timeout proporcional ao último período, limitado por MIN e MAX
            uint32_t timeout;

            if (periodo > (RPM_PULSE_TIMEOUT_MAX / RPM_PULSE_TIMEOUT_MULT)) {
                timeout = RPM_PULSE_TIMEOUT_MAX;
            } else {
                timeout = periodo * RPM_PULSE_TIMEOUT_MULT;
            }

            if (timeout < RPM_PULSE_TIMEOUT_MIN) {
                timeout = RPM_PULSE_TIMEOUT_MIN;
            }

            if (timeout > RPM_PULSE_TIMEOUT_MAX) {
                timeout = RPM_PULSE_TIMEOUT_MAX;
            }

            if (idadePulso <= timeout) {
                pulso_recente = true;
            }
        }

        if (pulso_recente && rpm_abs_periodo > 0.0f && rpm_abs_periodo < LIMIAR_LEITURA_RPM) {
            float rpm_raw = sentidoAtual * rpm_abs_periodo;

            rpm_raw = constrain(rpm_raw, -(float)RPM_MAX, (float)RPM_MAX);

            RPM_ant = RPM_FILT * RPM_ant + (1.0f - RPM_FILT) * rpm_raw;

            if (fabsf(RPM_ant) < RPM_BANDA_MORTA) {
                RPM_ant = 0.0f;
                sentidoAtual = sentidoSelecionado;
            }

            return RPM_ant;
        }

        /*
           Pulso expirado: motor desacelerando ou parado.
        */
        RPM_ant *= RPM_DECAIMENTO;

        if (fabsf(RPM_ant) < RPM_BANDA_MORTA) {
            RPM_ant = 0.0f;
            sentidoAtual = sentidoSelecionado;
        }

        return RPM_ant;
    }

    // ===================== Estimativa por janela ================== //

    /*
       Cálculo da velocidade por contagem de pulsos na janela.

       Essa estimativa é melhor em média/alta velocidade, pois há mais pulsos
       dentro da janela de 10 ms.
    */
    float time_s = TempoDecorrido_ms / 1000.0f;

    float rpm_abs_janela = (pulses_mod * 60.0f) / (time_s * N_FUROS);
    rpm_abs_janela = constrain(rpm_abs_janela, 0.0f, (float)RPM_MAX);

    float rpm_abs;

    if (periodo_valido && rpm_abs_periodo > 0.0f && rpm_abs_periodo < LIMIAR_LEITURA_RPM) {
        rpm_abs = rpm_abs_periodo;
    } else {
        rpm_abs = rpm_abs_janela;
    }

    float rpm_raw = sentidoAtual * rpm_abs;

    // Limita também a velocidade com sinal.
    rpm_raw = constrain(rpm_raw, -(float)RPM_MAX, (float)RPM_MAX);

    // ===================== Filtro e banda morta =================== //

    /*
       Filtro leve para reduzir degraus de medição e ruído.
    */
    RPM_ant = RPM_FILT * RPM_ant + (1.0f - RPM_FILT) * rpm_raw;

    // Evita que valores residuais muito pequenos fiquem alimentando o PI.
    if (fabsf(RPM_ant) < RPM_BANDA_MORTA) {
        RPM_ant = 0.0f;
    }

    return RPM_ant;
}

// ================================================================== //

// ===================== Corrente do motor ======================== //

float CorrenteMotor() {

    if (!ads_ok) {
        return 0.0f;
    }

    ads.setGain(ADS_GAIN_CORRENTE);

    int16_t leitura = ads.readADC_Differential_0_3();

    double vshunt = ads.computeVolts(leitura);
    vshunt = trunc(vshunt * 1000.0) / 1000.0;

    double corrente = ((fabs(vshunt - 0.001f)) / RSHUNT_OHMS) * 1.262; // 0.001f e 1.262 são valor de ajustes definidos empiricamente

    if (primeira_corrente) {
        corrente_filtrada = corrente;
        primeira_corrente = false;
    } else {
        corrente_filtrada = FILTRO_CORRENTE * corrente_filtrada +
                            (1.0f - FILTRO_CORRENTE) * corrente;
    }

    return corrente_filtrada;
}

// ================================================================== //

// ===================== Tensão do motor ========================== //

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

// ================================================================== //
