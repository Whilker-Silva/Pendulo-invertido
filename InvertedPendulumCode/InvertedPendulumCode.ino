#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==============================
// CONFIGURAÇÃO DOS PINOS
// ==============================
#define ENCODER_PEND_A 33
#define ENCODER_PEND_B 32
#define ENCODER_MOT_A 13
#define ENCODER_MOT_B 14

#define POTENCIOMETRO 34

#define MOTOR_PWM1 27
#define MOTOR_PWM2 26

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==============================
// VARIÁVEIS GLOBAIS
// ==============================
const float PERIODO = 10.0;  // 10 ms = 100 Hz

volatile long encoderPendCount = 0;
volatile long encoderMotCount = 0;

int lastEncodedPend = 0;
int lastEncodedMot = 0;

const int PULSOS_POR_REV_PEND = 600;      
const int PULSOS_POR_REV_MOT  = 16;

const int RESOLUCAO_PEND = PULSOS_POR_REV_PEND * 4;  // Quadratura 4x
const int RESOLUCAO_MOT  = PULSOS_POR_REV_MOT  * 4;  // Quadratura 4x

const float FATOR_CONV_DIST = 145.366; //Pulsos pos cm

// Variáveis do degrau
volatile bool degrauAtivo = false;
unsigned long tempoDegrauInicio = 0;
unsigned long duracaoDegrau = 100; // 500 ms
uint8_t intensidadeDegrau = 0;
char sentidoDegrau = 'R';

// Variáveis do seno
volatile bool senoideAtiva = false;
unsigned long tempoSenoInicio = 0;
float amplitudeSeno = 0;       // 0-255
float frequenciaSeno = 0.5;          // Hz
unsigned long duracaoSeno = 1000; // duração em ms

volatile char comandoManual = 'P';
volatile uint8_t pwmManual = 180;

// Variáveis do controlador
volatile bool controleAtivo = false;
float K[4] = {0, 0, 0, 0};
float K_swing = 45;
bool modoLQR = false;   // false = swing-up, true = LQR

// Limiares de troca
const float THETA_SWITCH = 12 * PI/180.0;       
const float THETA_DOT_SWITCH = 15 * PI/180.0;  
const float FIM_CURSO_VIRTUAL =  0.20; 

// Energia desejada do pêndulo ereto
const float m = 0.0205;       // use os seus valores!
const float l = 0.18;
const float g = 9.81;
const float I = 0.000207;
const float M = 0.3088;       // massa do carrinho (kg)
const float b = 0.000008;       // atrito do pêndulo
const float c = 6.0;       // atrito do carrinho
const float kt = 0.175;
const float kb = 0.04;
const float Rm = 10.5;
const float r  = 0.071;

// Estados;
float x = 0;
float x_dot = 0;
float theta = 0;
float theta_dot = 0;

float velPendFilt = 0, velMotFilt = 0;

float setpointPos = 0.0;


// ==============================
// INTERRUPÇÕES DOS ENCODERS
// ==============================
void IRAM_ATTR updateEncoderPend() {
  int MSB = digitalRead(ENCODER_PEND_A);
  int LSB = digitalRead(ENCODER_PEND_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedPend << 2) | encoded;

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
void aplicaDegrau() {
  if (degrauAtivo) {
    // Define PWM de acordo com o sentido
    if (sentidoDegrau == 'R') {
      ledcWrite(0, intensidadeDegrau);  // MOTOR_PWM1
      ledcWrite(1, 0);                  // MOTOR_PWM2
    } else {
      ledcWrite(0, 0);                  // MOTOR_PWM1
      ledcWrite(1, intensidadeDegrau);  // MOTOR_PWM2
    }

    // Verifica tempo decorrido
    if (millis() - tempoDegrauInicio >= duracaoDegrau) {
      degrauAtivo = false;           // desliga degrau
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}


// ==============================
// FUNÇÃO PARA CONTROLAR MANUALMENTE
// ==============================
void controleManual() {
  if (comandoManual == 'L') {
    ledcWrite(0, 0);
    ledcWrite(1, pwmManual);
  }
  else if (comandoManual == 'R') {
    ledcWrite(0, pwmManual);
    ledcWrite(1, 0);
  }
  else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}


// ==============================
// FUNÇÃO PARA CONTROLAR SENOIDE
// ==============================
void aplicaSenoide() {
  if (senoideAtiva) {
    float t = (millis() - tempoSenoInicio) / 1000.0;  // tempo em segundos
    float s = sin(2 * PI * frequenciaSeno * t);       // valor da senoide [-1,1]
    int pwm = (int)(amplitudeSeno * abs(s));          // PWM sempre positivo

    if (s >= 0) {
      ledcWrite(0, pwm);  // MOTOR_PWM1 frente
      ledcWrite(1, 0);    // MOTOR_PWM2 desligado
    } else {
      ledcWrite(0, 0);    // MOTOR_PWM1 desligado
      ledcWrite(1, pwm);  // MOTOR_PWM2 trás
    }

    // verifica se o tempo acabou
    if (millis() - tempoSenoInicio >= duracaoSeno) {
      senoideAtiva = false;
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }

  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

// ==============================
// CONTROLADOR POR SWING UP
// ==============================

float sign(float x_cop) {
    if (x_cop > 0) return 1.0;
    else if (x_cop < 0) return -1.0;
    else return 0.0;
}

float Force2Volt(float F) {
    float volt = (F * Rm * r * r + kt * kb * x_dot) / (kt * r);
    return volt;
}

float swingUpController() {
    float E = m*g*l*(1 - cos(theta)) + 0.5*(I + m*l*l)*(theta_dot*theta_dot);
    float E_des = 2*m*g*l;  // topo

    float arg = theta_dot * cos(theta);

    float k_energy = K_swing * g;

    float x_2dot_desejado = k_energy * (E - E_des) * sign(arg) - 8*x;
    
    float theta_2dot = (-b * theta_dot
                        - m * l * cos(theta) * x_2dot_desejado
                        - m * g * l * sin(theta))
                       / (I + m * l * l);

    // === Cálculo da força F no carrinho ===
    float F = (M + m) * x_2dot_desejado
                + m * l * cos(theta) * theta_2dot
                - m * l * sin(theta) * (theta_dot * theta_dot)
                + c * x_dot;

    return Force2Volt(F);
}


// ==============================
// FUNÇÃO PARA CONTROLE LQR
// ==============================
void controleEstado() {

  float erroX = x - (setpointPos / 100.0f);  // converte cm → metros, se sua pos está em m
  float erroTheta = theta - PI; 

  float u = 0;

  bool emZonaPerigo = abs(x) > FIM_CURSO_VIRTUAL;
  bool emRegiaoLQR = (abs(erroTheta) < THETA_SWITCH) && (abs(theta_dot) < THETA_DOT_SWITCH);

  if (emZonaPerigo){
    u = - K[3] * erroX;
  }else if(emRegiaoLQR){
    u = -(K[0]*erroX + K[1]*erroTheta + K[2]*x_dot + K[3]*theta_dot);
  }else{
    u = swingUpController();
  }

  float umax = 12.0;
  u = constrain(u, -umax, umax);
  float u_pwm = (u / umax) * 255.0;
  
  // Aplica o controle ao motor
  if (u_pwm >= 0) {
    ledcWrite(0, (int)u_pwm);
    ledcWrite(1, 0);
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, (int)(-u_pwm));
  }
}

// ==============================
// TAREFA DE AMOSTRAGEM (10 ms)
// ==============================
void taskLeitura(void *parameter) {
  const TickType_t periodo = pdMS_TO_TICKS(PERIODO);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Variáveis auxiliares para cálculo de velocidade
  float theta_ant = 0;
  float pos_ant  = 0;

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, periodo);

    if(controleAtivo){
      controleEstado();
    } else if(degrauAtivo){
      aplicaDegrau();
    } else if(senoideAtiva){
      aplicaSenoide();
    } else {
      controleManual();
    }

    float tempo_s = millis() / 1000.0;

    // Cópias locais (evita conflito com interrupções)
    long countMot = encoderMotCount;
    long countPend = encoderPendCount;

    x = (float)countMot / (FATOR_CONV_DIST * 100.0f);
    x_dot = (x - pos_ant) / (PERIODO / 1000.0);
    pos_ant  = x;

    // Ângulo entre 0 e 2π
    long idx = countPend % RESOLUCAO_PEND;
    if (idx < 0) idx += RESOLUCAO_PEND;  // garante faixa positiva

    theta = (idx * 2*PI) / RESOLUCAO_PEND;  // 0 a 2π

    // Cálculo correto da velocidade angular (trata salto 2π → 0)
    float theta_raw = theta - theta_ant;

    // Se houve passagem pelo zero:
    if (theta_raw > PI)       theta_raw -= 2*PI;
    else if (theta_raw < -PI) theta_raw += 2*PI;

    theta_dot = theta_raw / (PERIODO / 1000.0) ;

    theta_ant = theta;

    int leituraPot = analogRead(POTENCIOMETRO);
    setpointPos = ((float)leituraPot / 4095.0f) * 30.0f - 15.0f;

    //Serial.printf("%.4f;%.2f;%.2f;%.2f;%.2f\n", tempo_s, theta*180/PI, theta_dot, x*100, x_dot*100);
  }
}

// ======================================================================
// TAREFA DE ATUALIZAÇÃO DO DISPLAY OLED (LENTA)
// ======================================================================
void taskDisplay(void *parameter) {
    // Define um período mais longo para o display, por exemplo, 100ms (10Hz)
    const TickType_t displayPeriodo = pdMS_TO_TICKS(100); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, displayPeriodo);

        // Apenas chama a função de desenho e envio (display.display())
        atualizarDisplay();
    }
}

// ==============================
// VERIFICA COMANDOS VIA SERIAL
// ==============================
// ===========================================
void taskSerial(void *parameter) {
    String buffer = "";
    while (true) {
        while (Serial.available()) {
            char c = Serial.read();

            // === 1. PROCESSAMENTO DE LINHA (Terminador '\n' ou '\r') ===
            if (c == '\n' || c == '\r') {
                
                // Processa o buffer se ele não estiver vazio
                if (buffer.length() > 0) {
                    
                    // Converte o primeiro caractere para maiúsculo para facilitar a comparação
                    char comando = buffer.charAt(0);
                    if (comando >= 'a' && comando <= 'z') {
                        comando = comando - 32; // Converte para maiúsculo
                    }

                    // ===============================
                    // COMANDO DE DEGRAU (Ex: D,500,200,R)
                    // ===============================
                    if (comando == 'D') {
                        // A lógica original de substring é mantida aqui.
                        int idx1 = buffer.indexOf(',');
                        int idx2 = buffer.indexOf(',', idx1 + 1);
                        int idx3 = buffer.lastIndexOf(',');
                        
                        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
                            String t = buffer.substring(idx1 + 1, idx2);
                            String v = buffer.substring(idx2 + 1, idx3);
                            String s = buffer.substring(idx3 + 1);

                            // Note: A comparação de sentidoDegrau deve ser robusta, 
                            // assumindo que a string 's' pode ter espaços/terminadores
                            // remanescentes no original. Aqui, mantive a lógica original, 
                            // mas o ideal seria limpar 's' antes.
                            s.toUpperCase(); // Garante 'L' ou 'R' maiúsculo
                            
                            duracaoDegrau = t.toInt();
                            intensidadeDegrau = constrain(v.toInt(), 0, 255);
                            sentidoDegrau = (s == "L") ? 'L' : 'R'; // L ou R

                            degrauAtivo = true;
                            senoideAtiva = false; // Desativa a senoide se o degrau for ativado
                            tempoDegrauInicio = millis();
                            
                            Serial.printf("Comando Degrau recebido: Duração=%ldms, Intensidade=%d, Sentido=%c\n", 
                                          duracaoDegrau, intensidadeDegrau, sentidoDegrau);
                        } else {
                             Serial.println("Erro: Comando D com formato inválido.");
                        }
                    }

                    // ===============================
                    // COMANDO SENOIDE: S,amplitude,frequencia
                    // ===============================
                    else if (comando == 'S') { 
                      int idx1 = buffer.indexOf(',');
                      int idx2 = buffer.indexOf(',', idx1 + 1);
                      int idx3 = buffer.lastIndexOf(',');

                      if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
                          amplitudeSeno = constrain(buffer.substring(idx1 + 1, idx2).toInt(), 0, 255);
                          frequenciaSeno = buffer.substring(idx2 + 1, idx3).toFloat();
                          duracaoSeno = buffer.substring(idx3 + 1).toFloat();  // Novo: duração em ms ou s

                          senoideAtiva = true;
                          degrauAtivo = false; // Desativa o degrau se a senoide for ativada
                          tempoSenoInicio = millis();
                      }
                    }

                    else if (comando == 'Q') {
                        int idx1 = buffer.indexOf(',');
                        int idx2 = buffer.indexOf(',', idx1 + 1);
                        int idx3 = buffer.indexOf(',', idx2 + 1);
                        int idx4 = buffer.indexOf(',', idx3 + 1);
                        int idx5 = buffer.lastIndexOf(',');

                        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3 && idx5 > idx4) {
                            K[0] = buffer.substring(idx1 + 1, idx2).toFloat();
                            K[1] = buffer.substring(idx2 + 1, idx3).toFloat();
                            K[2] = buffer.substring(idx3 + 1, idx4).toFloat();
                            K[3] = buffer.substring(idx4 + 1, idx5).toFloat();
                            K_swing = buffer.substring(idx5 + 1).toFloat();

                            controleAtivo = true;
                            degrauAtivo = false;
                            senoideAtiva = false;

                            if(theta == 0){
                              ledcWrite(1, 200);
                            }
                        }
                    }

                    else if (comando == 'X') {
                        controleAtivo = false;  // desativa controle automático
                        ledcWrite(0, 0);        // motor para por segurança
                        ledcWrite(1, 0);
                    }

                    // ===============================
                    // COMANDOS SIMPLES: L / R / P (Nova Lógica Robusta)
                    // ===============================
                    // Verifica se o buffer tem APENAS 1 caractere (L, R ou P)
                    else if (buffer.length() == 1) { 
                        if (comando == 'L' || comando == 'R' || comando == 'P') {
                            comandoManual = comando; // Usa a versão maiúscula
                            
                            // Desativa degrau/senoide se um comando manual for enviado
                            degrauAtivo = false;
                            senoideAtiva = false;
                        } else if(comando == 'Z') {
                            encoderPendCount = 0;
                            encoderMotCount = 0;
                            lastEncodedPend = 0;
                            lastEncodedMot = 0;
                        }
                    }
                    
                    // Limpa o buffer após o processamento, independentemente do sucesso
                    buffer = "";
                }
            } 
            
            // === 2. CONSTRUÇÃO DO BUFFER (Não é terminador) ===
            else {
                // Adiciona o caractere ao buffer (Ignora '\n' e '\r' do input)
                buffer += c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequena pausa para permitir que outras tasks rodem
    }
}

// ======================================================================
//  DISPLAY OLED
// ======================================================================
void telaBoasVindas() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(20, 8);
  display.println("Projeto");

  display.setCursor(10, 22);
  display.println("Lab Integrador");

  display.setCursor(0, 48);
  display.println("Inicializando...");

  display.display();
  delay(2500);
}

void telaCalibrandoPendulo() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 8);
  display.println("Calibrando pendulo...");
  display.setCursor(0, 28);
  display.println("Deixe em repouso");

  display.display();
}

void atualizarDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ----- Titulo centralizado -----
  display.setTextSize(1);
  const char titulo[] = "Projeto LabIntegrador";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(titulo, 0, 0, &x1, &y1, &w, &h);
  int16_t xTitulo = (SCREEN_WIDTH - w) / 2;
  display.setCursor(xTitulo, 0);
  display.print(titulo);

  // Linha separadora
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

  // ----- Bloco MOTOR -----
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("Posição");

  display.setTextSize(2);
  display.setCursor(50, 24);
  display.print(x*100, 1);

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("cm");

  display.drawLine(0, 42, SCREEN_WIDTH - 1, 42, SSD1306_WHITE);

  // ----- Bloco PENDULO -----
  display.setTextSize(1);
  display.setCursor(0, 46);
  display.print("Set Point");

  display.setTextSize(2);
  display.setCursor(50, 52 - 8);
  display.print(setpointPos, 1);

  display.setTextSize(1);
  display.setCursor(0, 62 - 4);
  display.print("deg");

  display.display();
}

// ==============================
// CONFIGURAÇÃO INICIAL
// ==============================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("Falha ao iniciar display!");
    while (true) {}
  }

  telaBoasVindas();


  pinMode(ENCODER_PEND_A, INPUT);
  pinMode(ENCODER_PEND_B, INPUT);
  pinMode(ENCODER_MOT_A, INPUT);
  pinMode(ENCODER_MOT_B, INPUT);
  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);
  pinMode(POTENCIOMETRO, INPUT);


  // PWM MOTOR 1
  ledcSetup(0, 10000, 8);        // canal 0, 10kHz, 8 bits
  ledcAttachPin(MOTOR_PWM1, 0);

  // PWM MOTOR 2
  ledcSetup(1, 10000, 8);        // canal 1, 10kHz, 8 bits
  ledcAttachPin(MOTOR_PWM2, 1);

  // Leitura inicial
  //lastEncodedPend = (digitalRead(ENCODER_PEND_A) << 1) | digitalRead(ENCODER_PEND_B);
  //lastEncodedMot  = (digitalRead(ENCODER_MOT_A)  << 1) | digitalRead(ENCODER_MOT_B);

  // Interrupções
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_A), updateEncoderPend, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_B), updateEncoderPend, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_MOT_A),  updateEncoderMot,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_MOT_B),  updateEncoderMot,  CHANGE);

  // Cria tarefa FreeRTOS
  xTaskCreatePinnedToCore(taskLeitura, "TaskLeitura", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSerial,   "TaskSerial",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskDisplay, "TaskDisplay", 4096, NULL, 1, NULL, 0);

}

void loop() {
  // Loop vazio
}
