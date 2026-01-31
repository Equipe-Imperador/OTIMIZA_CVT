#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ========================== 1. DEFINIÇÃO DE PINOS ==========================
#define PIN_RPM          33
#define PIN_VEL_TRAS     32 
#define PIN_VEL_DIANT_E  34
#define PIN_VEL_DIANT_D  35
#define SD_CS            5

// ========================== 2. PARÂMETROS FÍSICOS ==========================
// Fatores de ajuste (Multiplicadores)
const float FATOR_RPM     = 1.0f;
const float FATOR_TRAS    = 1.0f; 
const float FATOR_DIANT_E = 1.0f;
const float FATOR_DIANT_D = 1.0f;

// Traseira
#define DIAMETRO_TRAS    0.538f   // Metros
#define DENTES_TRAS      3
#define REDUCAO_TRAS     9.0f
const float CIRCUM_TRAS = PI * DIAMETRO_TRAS;

// Dianteira
#define DIAMETRO_DIANT   0.5602f  // Metros
#define DENTES_DIANT     5 
const float CIRCUM_DIANT = PI * DIAMETRO_DIANT;

// Motor
#define DENTES_RPM       1 

// ========================== 3. FILTROS E CONSTANTES ========================
const int JANELA_SUP = 2000;         // Schmitt Trigger High
const int JANELA_INF = 1500;         // Schmitt Trigger Low

// Timings
const unsigned long TIMEOUT_US      = 500000; // 0.5s sem pulso = 0 km/h
const unsigned long DEBOUNCE_RPM_US = 2000;   // 2ms (Máx 30k RPM) - Filtra ruído da vela
const unsigned long DEBOUNCE_VEL_US = 1000;   // 1ms (Máx 60k Hz) - Sensor de roda

// ========================== 4. ESTRUTURAS E GLOBAIS ========================
struct SpeedPacket {
  uint32_t timestamp;
  float rpm;
  float vTras;
  float vDiantE;
  float vDiantD;
};

QueueHandle_t dataQueue;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Variáveis modificadas por interrupção (Volatile)
volatile unsigned long dtRPM = 0, lastRPM = 0;
volatile unsigned long dtTras = 0, lastTras = 0;

// ========================== 5. INTERRUPÇÕES (ISRs) =========================
// Otimização: micros() fora da seção crítica para não travar o Core
void IRAM_ATTR isrRPM() {
  unsigned long agora = micros(); 
  
  portENTER_CRITICAL_ISR(&timerMux);
  if (agora - lastRPM > DEBOUNCE_RPM_US) {
    dtRPM = agora - lastRPM;
    lastRPM = agora;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR isrTras() {
  unsigned long agora = micros();

  portENTER_CRITICAL_ISR(&timerMux);
  if (agora - lastTras > DEBOUNCE_VEL_US) {
    dtTras = agora - lastTras;
    lastTras = agora;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ========================== 6. FUNÇÕES DE CÁLCULO ==========================
float calcRPM(unsigned long dt) {
  if (dt == 0) return 0.0f;
  return (60000000.0f / (dt * DENTES_RPM)) * FATOR_RPM;
}

float calcVelTras(unsigned long dt) {
  if (dt == 0) return 0.0f;
  return ((1000000.0f * CIRCUM_TRAS * 3.6f) / (dt * REDUCAO_TRAS * DENTES_TRAS)) * FATOR_TRAS;
}

float calcVelDiant(unsigned long dt, float fator) {
  if (dt == 0) return 0.0f;
  return ((1000000.0f * CIRCUM_DIANT * 3.6f) / (dt * DENTES_DIANT)) * fator;
}

// ========================== 7. TASK SENSOR (CORE 1) ========================
void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  // Variáveis locais persistentes (Estado dos sensores dianteiros)
  unsigned long lastDiantE = 0, lastDiantD = 0;
  unsigned long dtDiantE = 0, dtDiantD = 0;
  int estE = 0, estD = 0;
  
  int sendCounter = 0; 

  for (;;) {
    unsigned long agora_us = micros();

    // --- LEITURA ANALÓGICA (Sensores Indutivos/Hall) ---
    int rawE = analogRead(PIN_VEL_DIANT_E);
    int rawD = analogRead(PIN_VEL_DIANT_D);

    // Dianteira Esquerda
    if (rawE > JANELA_SUP && estE == 0) {
      // Correção "Primeira Leitura": Só calcula se lastDiantE != 0
      if (lastDiantE != 0 && (agora_us - lastDiantE > DEBOUNCE_VEL_US)) {
        dtDiantE = agora_us - lastDiantE;
      }
      lastDiantE = agora_us;
      estE = 1;
    } else if (rawE < JANELA_INF) { estE = 0; }

    // Dianteira Direita
    if (rawD > JANELA_SUP && estD == 0) {
      if (lastDiantD != 0 && (agora_us - lastDiantD > DEBOUNCE_VEL_US)) {
        dtDiantD = agora_us - lastDiantD;
      }
      lastDiantD = agora_us;
      estD = 1;
    } else if (rawD < JANELA_INF) { estD = 0; }

    // --- TIMEOUT LOCAL (Dianteiras) ---
    // Como dtDiant são locais, podemos zerar sem risco de concorrência
    if (agora_us - lastDiantE > TIMEOUT_US) { dtDiantE = 0; lastDiantE = 0; }
    if (agora_us - lastDiantD > TIMEOUT_US) { dtDiantD = 0; lastDiantD = 0; }

    // --- PROCESSO DE ENVIO (Downsampling para 100Hz) ---
    if (++sendCounter >= 10) { 
      sendCounter = 0;
      SpeedPacket packet;
      unsigned long _dtR, _dtT;
      unsigned long _lastR, _lastT;

      // SNAPSHOT: Cópia atômica das variáveis globais
      // Não alteramos as globais aqui para evitar Race Condition
      portENTER_CRITICAL(&timerMux); 
      _dtR = dtRPM;     _lastR = lastRPM;
      _dtT = dtTras;    _lastT = lastTras;
      portEXIT_CRITICAL(&timerMux);  

      packet.timestamp = millis();

      // CÁLCULO SEGURO (Apenas na saída)
      // Se estourou o timeout, assumimos 0 no pacote, mas mantemos o dado bruto
      packet.rpm   = (agora_us - _lastR > TIMEOUT_US) ? 0.0f : calcRPM(_dtR);
      packet.vTras = (agora_us - _lastT > TIMEOUT_US) ? 0.0f : calcVelTras(_dtT);
      
      packet.vDiantE = calcVelDiant(dtDiantE, FATOR_DIANT_E);
      packet.vDiantD = calcVelDiant(dtDiantD, FATOR_DIANT_D);

      xQueueSend(dataQueue, &packet, 0);
    }

    // Mantém loop a 1kHz exatos
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1)); 
  }
}

// ========================== 8. TASK SD (CORE 0) ============================
void TaskSD(void *pvParameters) {
  File dataFile;
  char fileName[20];
  int flushCounter = 0;

  // Inicialização Segura
  if (!SD.begin(SD_CS)) {
    Serial.println("ERRO CRÍTICO: Falha no SD!");
    vTaskDelete(NULL); // Mata a task de log, mas sensores continuam rodando
  }

  // Cria arquivo incremental (log_v1, log_v2...)
  int n = 1;
  while (n < 1000) {
    sprintf(fileName, "/log_v%d.csv", n);
    if (!SD.exists(fileName)) break; 
    n++;
  }
  
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("time;rpm;vTras;vDiantE;vDiantD");
    Serial.printf("Log Iniciado: %s\n", fileName);
  }

  SpeedPacket p;
  for (;;) {
    // Bloqueia até receber dados da fila
    if (xQueueReceive(dataQueue, &p, portMAX_DELAY) == pdPASS) {
      if (dataFile) {
        dataFile.printf("%u;%.0f;%.2f;%.2f;%.2f\n", 
                        p.timestamp, p.rpm, p.vTras, p.vDiantE, p.vDiantD);
        
        // Flush a cada 50 amostras (0.5s) para segurança de dados
        if (++flushCounter >= 50) {
          dataFile.flush();
          flushCounter = 0;
        }
      }
    }
  }
}

// ========================== 9. SETUP =======================================
void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_RPM, INPUT);
  pinMode(PIN_VEL_TRAS, INPUT);
  pinMode(PIN_VEL_DIANT_E, INPUT);
  pinMode(PIN_VEL_DIANT_D, INPUT);

  // Interrupções para sensores de alta frequência
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_TRAS), isrTras, FALLING);

  // Fila para 100 amostras
  dataQueue = xQueueCreate(100, sizeof(SpeedPacket));

  // Fixando Tasks nos Cores para evitar conflito de Cache/WiFi
  // Core 1 (App): Cálculos intensivos e Sensores
  // Core 0 (Sys): Acesso ao SD e Sistema de Arquivos
  xTaskCreatePinnedToCore(TaskSensor, "Sensor", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskSD,     "SD",     4096, NULL, 2, NULL, 0);
  
  Serial.println("--- Baja Telemetria V5.0 (Final) ---");
}

void loop() { 
  vTaskDelete(NULL); // Libera memória da task main
}