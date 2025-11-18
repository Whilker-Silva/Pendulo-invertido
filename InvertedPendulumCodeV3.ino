// ==============================
// CONFIGURAÇÃO DOS PINOS
// ==============================
// Encoders
#define ENCODER_PEND_A 32
#define ENCODER_PEND_B 33
#define ENCODER_MOT_A 13
#define ENCODER_MOT_B 14

// Motor (PWM) - dois canais para direção da ponte H (L298N)
#define MOTOR_PWM1 27
#define MOTOR_PWM2 26

// Switches de limite (quando o carrinho encosta nas extremidades)
#define SWITCH_ESQ 15
#define SWITCH_DIR 4

// Switches de ligar/desligar do sistema (botões físicos)
#define SWITCH_ON  21
#define SWITCH_OFF 22

// Potenciômetro (entrada analógica)
#define POT_PIN 34   // ADC1_CH6 - valor 0..4095 (verificar range do pot) no ESP32

// LED de status
#define LED_STATUS 2

// ==============================
// VARIÁVEIS GLOBAIS / CONSTANTES
// ==============================
const float PERIODO = 10.0;  // 10 ms = 100 Hz (período da task de leitura)

volatile long encoderPendCount = 0;
volatile long encoderMotCount = 0;

int lastEncodedPend = 0;
int lastEncodedMot = 0;

const int PULSOS_POR_REV_PEND = 600;      // pulsos por volta do encoder do pêndulo (exemplo)
const int PULSOS_POR_REV_MOT  = 16;

const int RESOLUCAO_PEND = PULSOS_POR_REV_PEND * 4;  // quadratura 4x
const int RESOLUCAO_MOT  = PULSOS_POR_REV_MOT  * 4;  // quadratura 4x

const float FATOR_CONV_DIST = 146.233; // fator de conversão de contagem -> distância (exemplo)

// Variáveis do degrau
volatile bool degrauAtivo = false;
unsigned long tempoDegrauInicio = 0;
unsigned long duracaoDegrau = 100; // ms
volatile uint8_t intensidadeDegrau = 0; // 0..255
volatile char sentidoDegrau = 'R'; // 'R' = para frente, 'L' = para trás

// Variáveis do seno
volatile bool senoideAtiva = false;
unsigned long tempoSenoInicio = 0;
volatile float amplitudeSeno = 0;       // 0..255
volatile float frequenciaSeno = 0.5;    // Hz
volatile unsigned long duracaoSeno = 1000; // ms

// Controle manual
volatile char comandoManual = 'P'; // 'L', 'R', 'P'
volatile uint8_t pwmManual = 180;  // 0..255 (pwm padrão manual)

// Variáveis do controlador (LQR)
volatile bool controleAtivo = false;
float K[4] = {0, 0, 0, 0}; // ganhos (K0..K3) para u = -K*x

// Estados estimados
float pos = 0;     // posição do carrinho (unidade convertida)
float velMot = 0;  // velocidade do carrinho
float angle = 0;   // ângulo do pêndulo (graus)
float velPend = 0; // velocidade angular do pêndulo

// ==============================
// Variáveis para debounce (simples) para switches
// ==============================
struct Debounce {
  bool stableState;
  bool lastRaw;
  unsigned long lastChangeTime;
  unsigned long stableDelay; // ms
};

Debounce db_esq = {HIGH, HIGH, 0, 50};   // usando INPUT_PULLUP: HIGH = sem pressão
Debounce db_dir = {HIGH, HIGH, 0, 50};
Debounce db_on  = {HIGH, HIGH, 0, 50};
Debounce db_off = {HIGH, HIGH, 0, 50};

// HELPERS: Leitura ADC com escolha de bits e mapeamento p/ resistência e posição
// ==============================
// Estrutura de retorno para leitura do potenciômetro
struct PotReading {
  int raw;          // valor bruto do ADC (0..maxADC)
  float ohms;       // resistência calculada (0..POT_MAX_OHMS)
  float pos_cm;     // posição mapeada (0..GUIA_LENGTH_CM)
};

// Retorna o valor máximo do ADC dependendo de ADC_BITS
int getMaxAdcValue() {
  if (ADC_BITS == 11) return 2047;
  return 4095; // default 12 bits
}

// Leitura do potenciômetro:
// - analogRead() retorna 0..4095 (ESP32 padrão). 
// - a leitura é linear, foi mapeado proporcionalmente para resistência (0..10000 ohms).
// - em seguida foi mapeada resistância p/ posição em centímetros (0..70 cm).
PotReading leituraPotentiometro() {
  PotReading r;
  int rawFull = analogRead(POT_PIN); // 0..4095 (ESP32 ADC padrão)
  int maxADC = getMaxAdcValue();

  // Se o usuário pediu 11 bits, reduzimos proporcionalmente
  if (ADC_BITS == 11) {
    // converte 0..4095 -> 0..2047 mantendo proporcionalidade
    r.raw = (rawFull * maxADC) / 4095;
  } else {
    r.raw = rawFull;
  }

  // Mapeamento raw -> ohms (assumindo potenciômetro 0..POT_MAX_OHMS)
  // Como a leitura ADC é proporcional à tensão, e a tensão é proporcional à divisão do potenciômetro,
  // foi determinada leitura linear e foi mapeado diretamente. 
  r.ohms = ((float)r.raw / (float)maxADC) * POT_MAX_OHMS;

  // Mapear ohms -> posição ao longo do guia
  r.pos_cm = (r.ohms / POT_MAX_OHMS) * GUIA_LENGTH_CM;

  return r;
}

// ==============================
// INTERRUPÇÕES DOS ENCODERS
// ==============================
// ISRs: leitura quadratura (4x). Leve e rápido — por isso IRAM_ATTR.
void IRAM_ATTR updateEncoderPend() {
  int MSB = digitalRead(ENCODER_PEND_A);
  int LSB = digitalRead(ENCODER_PEND_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedPend << 2) | encoded;

  // Tabela de transições válidas para incremento/decremento (quadratura)
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPendCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPendCount--;

  lastEncodedPend = encoded;
}

void IRAM_ATTR updateEncoderMot() {
  int MSB = digitalRead(ENCODER_MOT_A);
  int LSB = digitalRead(ENCODER_MOT_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedMot << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderMotCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderMotCount--;

  lastEncodedMot = encoded;
}

// ==============================
// FUNÇÃO PARA CONTROLAR DEGRAU
// ==============================
// Descrição detalhada:
// - Quando degrauAtivo == true, é aplicado um PWM fixo (intensidadeDegrau) por duracaoDegrau ms.
// - sentidoDegrau decide o sentido: 'R' -> motor_pwm1 ativo (frente), 'L' -> motor_pwm2 ativo (ré).
// - Após expirar o tempo, o motor para e desfazemos degrauAtivo.
// - A função é chamada periodicamente (taskLeitura) e é não-bloqueante (não usa função delay que congela o processador).
void aplicaDegrau() {
  if (degrauAtivo) {
    // Aplicar PWM conforme sentido
    if (sentidoDegrau == 'R') {
      // Faz motor girar para frente: canal 0 com intensidade
      ledcWrite(0, intensidadeDegrau); // MOTOR_PWM1
      ledcWrite(1, 0);                // MOTOR_PWM2 desligado
    } else {
      // Sentido contrário: canal 1 ativo
      ledcWrite(0, 0);
      ledcWrite(1, intensidadeDegrau);
    }

    // Verifica se o tempo já passou (não bloqueante)
    if (millis() - tempoDegrauInicio >= duracaoDegrau) {
      // fim do degrau: limpar flags e parar motor
      degrauAtivo = false;
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      // Nota: não foi alterado o 'comandoManual' - assumimos que ao término do degrau,
      // o sistema volta ao estado que já estivesse selecionado (manual, controle, etc).
    }
  } else {
    // Se degrau não ativo, garantir que motor não seja atropelado por esta função
    // (a lógica externa decide o que deve aplicar)
  }
}

// ==============================
// FUNÇÃO PARA CONTROLAR MANUALMENTE
// ==============================
// Usa 'comandoManual' e 'pwmManual' (este pode vir do potenciômetro)
void controleManual() {
  if (comandoManual == 'L') {
    // sentido esquerda (ajuste conforme sua H-bridge)
    ledcWrite(0, 0);
    ledcWrite(1, pwmManual);
  }
  else if (comandoManual == 'R') {
    ledcWrite(0, pwmManual);
    ledcWrite(1, 0);
  }
  else {
    // 'P' = parado
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

// ==============================
// FUNÇÃO PARA CONTROLAR SENOIDE
// ==============================
// Descrição detalhada:
// - Gera uma referência senoidal em amplitude (0..255) e frequência (Hz).
// - A função calcula s = sin(2*pi*f*t), com t em segundos a partir do inicio.
// - Usa abs(s) para PWM (PWM só aceita valores positivos), mas o sinal do seno
//   decide o sentido (frente ou trás).
// - A senoide é aplicada por 'duracaoSeno' ms a partir de 'tempoSenoInicio'.

void aplicaSenoide() {
  if (senoideAtiva) {
    float t = (millis() - tempoSenoInicio) / 1000.0;  // tempo em segundos desde início
    float s = sin(2.0 * PI * frequenciaSeno * t);     // valor -1..1
    int pwm = (int)(amplitudeSeno * fabs(s));        // magnitude do PWM (0..amplitude)

    // Decisão de sentido baseada no sinal da senoide:
    if (s >= 0.0f) {
      // fase positiva: frente
      ledcWrite(0, pwm);  // MOTOR_PWM1
      ledcWrite(1, 0);
    } else {
      // fase negativa: ré
      ledcWrite(0, 0);
      ledcWrite(1, pwm);
    }

    // Verifica duração
    if (millis() - tempoSenoInicio >= duracaoSeno) {
      // acabou a execução da senoide
      senoideAtiva = false;
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
  } else {
    // nada a fazer aqui
  }
}

// ==============================
// FUNÇÃO PARA CONTROLE LQR (CONTROLE POR ESTADO)
// ==============================
// Descrição detalhada:
// - Estados x = [angle; velPend; pos; velMot]
// - Lei de controle linear u = -K * x
// - K[0] = ganho para angle, K[1] para velPend, K[2] para pos, K[3] para velMot
// - u é saturado em -255..255 (para converter p/ PWM 8 bits).
// - Se u >= 0: aplica em MOTOR_PWM1; se u < 0: aplica em MOTOR_PWM2 com valor |u|.
// - É importante que os estados estejam nas mesmas unidades em que K foi projetado.
// -  Se os ganhos vierem do projeto em radianos, converter angle -> rad (angle * PI/180).
void controleEstado() {
  // Calcular lei de controle linear
  // Atenção: verifique a unidade de 'angle' que foi utilizada ao projetar K.
  float u = -(K[0]*angle + K[1]*velPend + K[2]*pos + K[3]*velMot);

  // Saturação
  if (u > 255.0f) u = 255.0f;
  if (u < -255.0f) u = -255.0f;

  // Aplicar no motor
  if (u >= 0.0f) {
    ledcWrite(0, (int)u);
    ledcWrite(1, 0);
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, (int)(-u));
  }
}

// ==============================
// FUNÇÃO DE DEBOUNCE (GENÉRICA PARA BOTÕES) - NÃO BLOQUEANTE
// ==============================
// - lê o estado bruto do pino (digitalRead), compara com a última leitura bruta.
// - se mudar, atualiza lastChangeTime; se estiver estável por stableDelay ms,
//   atualiza stableState.
// - retorna o estado estável atual (HIGH/LOW).
bool debounceRead(int pin, Debounce &db) {
  bool raw = digitalRead(pin);
  unsigned long now = millis();

  if (raw != db.lastRaw) {
    db.lastChangeTime = now;
    db.lastRaw = raw;
  }

  if ((now - db.lastChangeTime) >= db.stableDelay) {
    // se estiver estável por tempo suficiente, atualiza stableState
    db.stableState = raw;
  }

  return db.stableState;
}

// ==============================
// FUNÇÃO QUE VERIFICA LIMITES (SWITCH ESQ/DIR)
// ==============================
// Se qualquer switch de limite for acionado, o motor para imediatamente.
// Também podemos definir comportamento de segurança adicional já que será feito eletricamente (ex: setar flags e mostrar no display).
void verificaLimites() {
  bool esq = debounceRead(SWITCH_ESQ, db_esq);
  bool dir = debounceRead(SWITCH_DIR, db_dir);

  // Considerando INPUT_PULLUP: LOW = botão pressionado / contato fechado
  if (esq == LOW) {
    // acionou limite esquerdo
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    Serial.println("Limite ESQUERDO atingido - motor parado");
    // opcional: desativar controle automático para evitar reativação
    controleAtivo = false;
  }
  if (dir == LOW) {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    Serial.println("Limite DIREITO atingido - motor parado");
    controleAtivo = false;
  }
}

// ==============================
// FUNÇÃO QUE VERIFICA SWITCH ON/OFF
// ==============================
// Lê botões de ligar/desligar e atualiza 'controleAtivo'.
// Se SWITCH_OFF pressionado, para o motor e desativa controle.
void verificaPower() {
  bool on  = debounceRead(SWITCH_ON, db_on);
  bool off = debounceRead(SWITCH_OFF, db_off);

  // INPUT_PULLUP: LOW = pressionado
  if (on == LOW) {
    if (!controleAtivo) {
      controleAtivo = true;
      Serial.println("Sistema LIGADO (controle ativado)");
    }
  }
  if (off == LOW) {
    if (controleAtivo) {
      controleAtivo = false;
    }
    // parar motor por segurança
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    Serial.println("Sistema DESLIGADO (controle desativado)");
  }
}

// ==============================
// LEITURA DO POTENCIÔMETRO
// ==============================
// Converte leitura ADC (0..4095) verificar range do pot. para 0..255
uint8_t leituraPOT() {
  int raw = analogRead(POT_PIN); // 0..4095 (ADC1)
  // Mapear para 0..255
  int p = map(raw, 0, 4095, 0, 255);
  if (p < 0) p = 0;
  if (p > 255) p = 255;
  return (uint8_t)p;
}

// ==============================
// TAREFA DE AMOSTRAGEM (10 ms)
// ==============================
// - Executa a malha de aquisição, cálculo de estados e aplicação do controle
// - Roda sempre a cada PERIODO ms usando vTaskDelayUntil (temporização precisa)
void taskLeitura(void *parameter) {
  const TickType_t periodo = pdMS_TO_TICKS(PERIODO);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // auxiliares para derivada (velocidade)
  float angleAnt = 0;
  float posAnt   = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, periodo);

    // 1) Verificações de hardware (switches, potencia)
    verificaLimites();
    verificaPower();

    // Atualiza pwmManual usando POT (se comandoManual estiver em modo manual)
    pwmManual = leituraPOT();

    // 2) Escolher o modo de atuação
    if (controleAtivo) {
      // Controle LQR automático (ganhos K já configurados via serial)
      controleEstado();
    } else if (degrauAtivo) {
      // Modo degrau (com tempo limitado)
      aplicaDegrau();
    } else if (senoideAtiva) {
      // Modo senoide (gera referência senoidal por uma duração)
      aplicaSenoide();
    } else {
      // Modo manual (usuário controla com L/R/P e potenciômetro)
      controleManual();
    }

    // 3) Leitura de sensores para estados
    float tempo_s = millis() / 1000.0;

    // Cópias locais (evita leituras diretas de variáveis voláteis múltiplas vezes)
    long countMot = encoderMotCount;
    long countPend = encoderPendCount;

    // posição do carrinho (conversão por fator)
    pos = (float)countMot / FATOR_CONV_DIST;

    // velocidade do carrinho (diferença / período)
    velMot = (pos - posAnt) * 1000.0f / PERIODO; // unidade por segundo
    posAnt = pos;

    // ângulo do pêndulo: converter contagem p/ graus
    // atenção: usamos o módulo RESOLUCAO_PEND para limitar faixa
    int contModulo = (int)(countPend % RESOLUCAO_PEND);
    if (contModulo < 0) contModulo += RESOLUCAO_PEND; // evitar negativo no modulo
    angle = ((contModulo * 360.0f) / RESOLUCAO_PEND) - 180.0f; // faixa -180..+180
    // garantir limite
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;

    // velocidade angular do pêndulo (diferença / período)
    velPend = (angle - angleAnt) * 1000.0f / PERIODO; // graus por segundo
    angleAnt = angle;

    // 4) Estado e telemetria via Serial
    // Formato CSV: tempo;angle;velPend;pos;velMot
    Serial.printf("%.4f;%.2f;%.2f;%.2f;%.2f\n", tempo_s, angle, velPend, pos, velMot);

    // 5) Atualizar LED de status
    if (controleAtivo)
      digitalWrite(LED_STATUS, HIGH);
    else
      digitalWrite(LED_STATUS, LOW);
 }
}

// ==============================
// TAREFA DE SERIAL (COMANDOS)
// ==============================
// Recebe comandos pela serial, como já estava no seu código (D, S, Q, L/R/P, X, Z).
void taskSerial(void *parameter) {
  String buffer = "";
  Serial.println("TaskSerial ativa (aguardando comandos)");

  for (;;) {
    // Processa dados seriais quando existirem
    while (Serial.available()) {
      char c = Serial.read();

      // Terminador de linha: processa comando
      if (c == '\n' || c == '\r') {
        if (buffer.length() > 0) {
          // pega o primeiro caractere do buffer e força maiúscula
          char comando = buffer.charAt(0);
          if (comando >= 'a' && comando <= 'z') comando = comando - 32;

          // ===== COMANDO DEGRAU: D,tempo_ms,intensidade(0-255),L/R
          if (comando == 'D') {
            int idx1 = buffer.indexOf(',');
            int idx2 = buffer.indexOf(',', idx1 + 1);
            int idx3 = buffer.lastIndexOf(',');

            if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
              String t = buffer.substring(idx1 + 1, idx2);
              String v = buffer.substring(idx2 + 1, idx3);
              String s = buffer.substring(idx3 + 1);
              s.trim();
              s.toUpperCase();

              unsigned long t_ms = (unsigned long)t.toInt();
              uint8_t val = (uint8_t)constrain(v.toInt(), 0, 255);
              char sentido = (s == "L") ? 'L' : 'R';

              // Setar variáveis voláteis
              duracaoDegrau = t_ms;
              intensidadeDegrau = val;
              sentidoDegrau = sentido;
              tempoDegrauInicio = millis();
              degrauAtivo = true;
              senoideAtiva = false; // garantir exclusão mútua
              Serial.printf("Degrau: %lums, intensidade=%d, sentido=%c\n", duracaoDegrau, intensidadeDegrau, sentidoDegrau);
            } else {
              Serial.println("Erro: formato comando D inválido. Exemplo: D,500,200,R");
            }
          }

          // ===== COMANDO SENOIDE: S,amplitude(0-255),frequencia(Hz),duracao_ms
          else if (comando == 'S') {
            int idx1 = buffer.indexOf(',');
            int idx2 = buffer.indexOf(',', idx1 + 1);
            int idx3 = buffer.lastIndexOf(',');

            if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
              amplitudeSeno = (float)constrain(buffer.substring(idx1 + 1, idx2).toInt(), 0, 255);
              frequenciaSeno = buffer.substring(idx2 + 1, idx3).toFloat();
              duracaoSeno = (unsigned long)buffer.substring(idx3 + 1).toInt();

              senoideAtiva = true;
              degrauAtivo = false;
              tempoSenoInicio = millis();
              Serial.printf("Senoide: amp=%.1f freq=%.2fHz dur=%lums\n", amplitudeSeno, frequenciaSeno, duracaoSeno);
            } else {
              Serial.println("Erro: formato comando S inválido. Ex: S,128,0.5,2000");
            }
          }

          // ===== COMANDO LQR: Q,k0,k1,k2,k3
          else if (comando == 'Q') {
            int idx1 = buffer.indexOf(',');
            int idx2 = buffer.indexOf(',', idx1 + 1);
            int idx3 = buffer.indexOf(',', idx2 + 1);
            int idx4 = buffer.lastIndexOf(',');

            if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3) {
              K[0] = buffer.substring(idx1 + 1, idx2).toFloat();
              K[1] = buffer.substring(idx2 + 1, idx3).toFloat();
              K[2] = buffer.substring(idx3 + 1, idx4).toFloat();
              K[3] = buffer.substring(idx4 + 1).toFloat();

              controleAtivo = true;
              degrauAtivo = false;
              senoideAtiva = false;
              Serial.printf("LQR ativo - K: %.4f, %.4f, %.4f, %.4f\n", K[0], K[1], K[2], K[3]);
            } else {
              Serial.println("Erro: formato comando Q inválido. Ex: Q,1.2,0.3,0.5,0.1");
            }
          }

          // ===== COMANDO X: desativa controle automático (safe stop)
          else if (comando == 'X') {
            controleAtivo = false;
            ledcWrite(0, 0);
            ledcWrite(1, 0);
            Serial.println("Comando X recebido: controle desativado e motor parado.");
          }

          // ===== COMANDOS SIMPLES L/R/P/Z
          else if (buffer.length() == 1) {
            if (comando == 'L' || comando == 'R' || comando == 'P') {
              comandoManual = comando;
              degrauAtivo = false;
              senoideAtiva = false;
              Serial.printf("Modo manual: %c (pwm=%d)\n", comandoManual, pwmManual);
            } else if (comando == 'Z') {
              // Zera encoders e estados auxiliares
              encoderPendCount = 0;
              encoderMotCount = 0;
              lastEncodedPend = 0;
              lastEncodedMot = 0;
              Serial.println("Encoders zerados");
            }
          }

          // Limpa buffer sempre que terminar processamento
          buffer = "";
        }
      } else {
        // Construção do buffer (caractere normal)
        buffer += c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // pequena pausa para permitir escalonamento
  }
}

// ==============================
// SETUP INICIAL
// ==============================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Configura pinos de encoder
  pinMode(ENCODER_PEND_A, INPUT);
  pinMode(ENCODER_PEND_B, INPUT);
  pinMode(ENCODER_MOT_A, INPUT);
  pinMode(ENCODER_MOT_B, INPUT);

  // Configura pinos do motor (saída)
  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);

  // PWM (ledc) - canal, freq, resolução
  ledcSetup(0, 1000, 8);        // canal 0, 1kHz, 8 bits
  ledcAttachPin(MOTOR_PWM1, 0);
  ledcSetup(1, 1000, 8);        // canal 1, 1kHz, 8 bits
  ledcAttachPin(MOTOR_PWM2, 1);

  // Configura switches com pullup interno (botões para GND)
  pinMode(SWITCH_ESQ, INPUT_PULLUP);
  pinMode(SWITCH_DIR, INPUT_PULLUP);
  pinMode(SWITCH_ON, INPUT_PULLUP);
  pinMode(SWITCH_OFF, INPUT_PULLUP);

  // Potenciômetro (entrada ADC)
  pinMode(POT_PIN, INPUT);

  // LED de status
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  // Inicializa variáveis de encoder (estado atual dos pinos)
  lastEncodedPend = (digitalRead(ENCODER_PEND_A) << 1) | digitalRead(ENCODER_PEND_B);
  lastEncodedMot  = (digitalRead(ENCODER_MOT_A)  << 1) | digitalRead(ENCODER_MOT_B);

  // Attach interrupts - CHANGE para capturar subida/descida em quadratura
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_A), updateEncoderPend, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_B), updateEncoderPend, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_MOT_A),  updateEncoderMot,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_MOT_B),  updateEncoderMot,  CHANGE);

  // Cria tasks FreeRTOS (pinned to core 1 para amostragem consistente pondedo ser 0 e 1 também devido ao multhread)
  xTaskCreatePinnedToCore(taskLeitura, "TaskLeitura", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSerial,  "TaskSerial",  4096, NULL, 1, NULL, 1);

  Serial.println("Setup concluído. Sistema pronto.");
}

void loop() {
  // Loop vazio porque todo trabalho é feito via FreeRTOS tasks
}
