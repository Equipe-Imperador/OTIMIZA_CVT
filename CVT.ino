#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// =================================================================
// ========================== CONFIGURAÇÕES ========================
// =================================================================

// Pinos dos Sensores
#define PIN_VEL_TRAS     32
#define PIN_VEL_DIANT_E  33
#define PIN_VEL_DIANT_D  25
#define SD_CS            5

// --- CONFIGURAÇÃO EIXO TRASEIRO ---
#define DIAMETRO_TRAS        0.54f
#define DENTES_TRAS          3
#define REDUCAO_TRAS         9.5f
const float CIRCUM_TRAS = 3.14159f * DIAMETRO_TRAS;

// --- CONFIGURAÇÃO EIXO DIANTEIRO ---
#define DIAMETRO_DIANT       0.52f
#define DENTES_DIANT         1
const float CIRCUM_DIANT = 3.14159f * DIAMETRO_DIANT;

// Parâmetros de Filtro
const unsigned long TIMEOUT_US = 500000;    
const unsigned long DEBOUNCE_MICROS_V = 1000; 

// Estrutura de Dados para a Fila
struct SpeedPacket {
  uint32_t timestamp;
  float vTras;
  float vDiantE;
  float vDiantD;
};

// Variáveis Globais e RTOS
QueueHandle_t dataQueue;
File dataFile;

// Variáveis de Interrupção (Voláteis)
volatile unsigned long dtTras = 0, lastTras = 0;
volatile unsigned long dtDiantE = 0, lastDiantE = 0;
volatile unsigned long dtDiantD = 0, lastDiantD = 0;

// =================================================================
// ======================== INTERRUPÇÕES ===========================
// =================================================================

void IRAM_ATTR isrTras() {
  unsigned long agora = micros();
  if (agora - lastTras < DEBOUNCE_MICROS_V) return;
  dtTras = agora - lastTras;
  lastTras = agora;
}

void IRAM_ATTR isrDiantE() {
  unsigned long agora = micros();
  if (agora - lastDiantE < DEBOUNCE_MICROS_V) return;
  dtDiantE = agora - lastDiantE;
  lastDiantE = agora;
}

void IRAM_ATTR isrDiantD() {
  unsigned long agora = micros();
  if (agora - lastDiantD < DEBOUNCE_MICROS_V) return;
  dtDiantD = agora - lastDiantD;
  lastDiantD = agora;
}

// =================================================================
// ====================== FUNÇÕES DE CÁLCULO =======================
// =================================================================

float calcVelTras(unsigned long dt) {
  if (dt == 0) return 0.0f;
  return (1000000.0f * CIRCUM_TRAS * 3.6f) / (dt * REDUCAO_TRAS * DENTES_TRAS);
}

float calcVelDiant(unsigned long dt) {
  if (dt == 0) return 0.0f;
  return (1000000.0f * CIRCUM_DIANT * 3.6f) / (dt * DENTES_DIANT);
}

// =================================================================
// ========================== TAREFAS RTOS =========================
// =================================================================

// CORE 1: Processamento de Velocidade
void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz

  for (;;) {
    SpeedPacket packet;
    unsigned long _dtT, _dtDE, _dtDD;
    unsigned long _lastT, _lastDE, _lastDD;
    unsigned long agora_us = micros();

    // Proteção para leitura das voláteis
    noInterrupts();
    _dtT = dtTras;     _lastT = lastTras;
    _dtDE = dtDiantE;  _lastDE = lastDiantE;
    _dtDD = dtDiantD;  _lastDD = lastDiantD;
    interrupts();

    packet.timestamp = millis();
    packet.vTras   = (agora_us - _lastT > TIMEOUT_US) ? 0.0f : calcVelTras(_dtT);
    packet.vDiantE = (agora_us - _lastDE > TIMEOUT_US) ? 0.0f : calcVelDiant(_dtDE);
    packet.vDiantD = (agora_us - _lastDD > TIMEOUT_US) ? 0.0f : calcVelDiant(_dtDD);

    // Envia para a fila (Core 0 vai ler)
    xQueueSend(dataQueue, &packet, 0);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// CORE 0: Gravação no SD
void TaskSD(void *pvParameters) {
  SpeedPacket receivedPacket;
  int counter = 0;

  for (;;) {
    if (xQueueReceive(dataQueue, &receivedPacket, portMAX_DELAY) == pdPASS) {
      if (dataFile) {
        dataFile.printf("%u;%.1f;%.1f;%.1f\n", 
                        receivedPacket.timestamp, 
                        receivedPacket.vTras, 
                        receivedPacket.vDiantE, 
                        receivedPacket.vDiantD);
        
        counter++;
        // Flush a cada 100 linhas para garantir que os dados sejam salvos
        if (counter >= 100) {
          dataFile.flush();
          counter = 0;
        }
      }
    }
  }
}

// =================================================================
// ============================ SETUP ==============================
// =================================================================

void setup() {
  Serial.begin(115200);

  // Configuração dos Pinos
  pinMode(PIN_VEL_TRAS, INPUT_PULLUP);
  pinMode(PIN_VEL_DIANT_E, INPUT_PULLUP);
  pinMode(PIN_VEL_DIANT_D, INPUT_PULLUP);

  // Inicializa SD
  if (!SD.begin(SD_CS)) {
    Serial.println("Erro ao iniciar SD!");
    while(1);
  }
  
  dataFile = SD.open("/velocidades.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("timestamp_ms;vTras;vDiantE;vDiantD");
    Serial.println("Arquivo SD aberto com sucesso.");
  }

  // Interrupções
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_TRAS), isrTras, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_E), isrDiantE, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_D), isrDiantD, FALLING);

  // Fila para 100 pacotes
  dataQueue = xQueueCreate(100, sizeof(SpeedPacket));

  if (dataQueue != NULL) {
    // TaskSensor no CORE 1 (Prioridade 3)
    xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 3, NULL, 1);
    
    // TaskSD no CORE 0 (Prioridade 2)
    xTaskCreatePinnedToCore(TaskSD, "TaskSD", 4096, NULL, 2, NULL, 0);
  }
}

void loop() {
  // Loop vazio - RTOS assume o controle
  vTaskDelay(pdMS_TO_TICKS(1000));
}
