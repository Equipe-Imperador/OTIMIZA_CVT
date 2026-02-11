#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ========================== CONFIGURAÇÕES FÍSICAS ========================
#define PIN_RPM          33
#define PIN_VEL_TRAS     32 
#define PIN_VEL_DIANT_E  34
#define PIN_VEL_DIANT_D  35
#define SD_CS            5

// --- CONFIGURAÇÃO MECÂNICA ---
#define DENTES_RPM       1   // Pulsos por volta do motor
#define DENTES_TRAS      3   // Imãs no disco traseiro
#define DENTES_DIANT     1   // Imãs nos discos dianteiros
#define REDUCAO_TRAS     9.0f
#define DIAMETRO_TRAS    0.54f
#define DIAMETRO_DIANT   0.52f

const float CIRCUM_TRAS  = 3.14159f * DIAMETRO_TRAS;
const float CIRCUM_DIANT = 3.14159f * DIAMETRO_DIANT;

// --- AJUSTES DE TIMEOUT E DEBOUNCE ---
const unsigned long DEBOUNCE_RPM = 11000;   // ~5400 RPM Max
const unsigned long DEBOUNCE_VEL = 2000;    // Filtro para as rodas
const unsigned long TIMEOUT_US   = 500000;  // 0.5s sem pulso = 0 RPM/Vel

// ========================== ESTRUTURAS E GLOBAIS =========================
struct SpeedPacket {
  uint32_t timestamp;
  float rpm;
  float vTras;
  float vDiantE;
  float vDiantD;
};

QueueHandle_t dataQueue;
File dataFile;

// Variáveis de Interrupção (Volatile)
volatile unsigned long v_dtRPM = 0, v_lastRPM = 0;
volatile unsigned long v_dtTras = 0, v_lastTras = 0;
volatile unsigned long v_dtDiantE = 0, v_lastDiantE = 0;
volatile unsigned long v_dtDiantD = 0, v_lastDiantD = 0;

// ======================== INTERRUPÇÕES (ISR) =============================
void IRAM_ATTR isrRPM() {
  unsigned long agora = micros();
  unsigned long diff = agora - v_lastRPM;
  if (diff < DEBOUNCE_RPM) return;
  v_dtRPM = diff;
  v_lastRPM = agora;
}

void IRAM_ATTR isrTras() {
  unsigned long agora = micros();
  unsigned long diff = agora - v_lastTras;
  if (diff < DEBOUNCE_VEL) return;
  v_dtTras = diff;
  v_lastTras = agora;
}

void IRAM_ATTR isrDiantE() {
  unsigned long agora = micros();
  unsigned long diff = agora - v_lastDiantE;
  if (diff < DEBOUNCE_VEL) return;
  v_dtDiantE = diff;
  v_lastDiantE = agora;
}

void IRAM_ATTR isrDiantD() {
  unsigned long agora = micros();
  unsigned long diff = agora - v_lastDiantD;
  if (diff < DEBOUNCE_VEL) return;
  v_dtDiantD = diff;
  v_lastDiantD = agora;
}

// ========================== TAREFAS RTOS =================================

// TAREFA 1: Processamento de Sensores (Core 1)
void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    SpeedPacket packet;
    unsigned long agora_us = micros();
    unsigned long dR, dT, dDE, dDD;

    // SEÇÃO CRÍTICA: Copia e reseta os valores voláteis
    noInterrupts();
    dR = (agora_us - v_lastRPM > TIMEOUT_US) ? 0 : v_dtRPM;
    dT = (agora_us - v_lastTras > TIMEOUT_US) ? 0 : v_dtTras;
    dDE = (agora_us - v_lastDiantE > TIMEOUT_US) ? 0 : v_dtDiantE;
    dDD = (agora_us - v_lastDiantD > TIMEOUT_US) ? 0 : v_dtDiantD;
    
    // Zera os DTs para garantir que não calcularemos a mesma volta duas vezes
    v_dtRPM = 0; v_dtTras = 0; v_dtDiantE = 0; v_dtDiantD = 0;
    interrupts();

    packet.timestamp = millis();
    
    // Cálculos de RPM e Velocidade (km/h)
    packet.rpm    = (dR > 0) ? (60000000.0f / (dR * DENTES_RPM)) : 0.0f;
    packet.vTras  = (dT > 0) ? ((1000000.0f * CIRCUM_TRAS * 3.6f) / (dT * REDUCAO_TRAS * DENTES_TRAS)) : 0.0f;
    packet.vDiantE = (dDE > 0) ? ((1000000.0f * CIRCUM_DIANT * 3.6f) / (dDE * DENTES_DIANT)) : 0.0f;
    packet.vDiantD = (dDD > 0) ? ((1000000.0f * CIRCUM_DIANT * 3.6f) / (dDD * DENTES_DIANT)) : 0.0f;

    xQueueSend(dataQueue, &packet, 0);
    
    // Roda a 50Hz (20ms) - ideal para dinâmica de CVT
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); 
  }
}

// TAREFA 2: Gravação no Cartão SD (Core 0)
void TaskSD(void *pvParameters) {
  SpeedPacket p;
  int counter = 0;
  for (;;) {
    if (xQueueReceive(dataQueue, &p, portMAX_DELAY) == pdPASS) {
      if (dataFile) {
        dataFile.printf("%u;%.0f;%.1f;%.1f;%.1f\n", p.timestamp, p.rpm, p.vTras, p.vDiantE, p.vDiantD);
        
        // Flush a cada 100 linhas para não estressar o Core 0
        if (++counter >= 100) {
          dataFile.flush();
          counter = 0;
        }
      }
    }
  }
}

// ============================ SETUP ======================================
void setup() {
  Serial.begin(115200);

  // Sensores com Pullup para maior imunidade a ruído
  pinMode(PIN_RPM, INPUT_PULLUP);
  pinMode(PIN_VEL_TRAS, INPUT_PULLUP);
  pinMode(PIN_VEL_DIANT_E, INPUT_PULLUP);
  pinMode(PIN_VEL_DIANT_D, INPUT_PULLUP);

  // Inicialização do SD
  if (!SD.begin(SD_CS)) {
    Serial.println("FALHA NO SD!");
  } else {
    char name[20];
    int n = 1;
    while (n < 1000) {
      sprintf(name, "/cvt_log%d.csv", n);
      if (!SD.exists(name)) break;
      n++;
    }
    dataFile = SD.open(name, FILE_WRITE);
    if (dataFile) {
      dataFile.println("time_ms;rpm;vel_tras;vel_diant_e;vel_diant_d");
      dataFile.flush();
      Serial.print("Arquivo criado: "); Serial.println(name);
    }
  }

  // Ativação das Interrupções
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_TRAS), isrTras, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_E), isrDiantE, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_D), isrDiantD, FALLING);

  // Criação da Fila e Tasks
  dataQueue = xQueueCreate(200, sizeof(SpeedPacket));
  
  xTaskCreatePinnedToCore(TaskSensor, "Sensor", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskSD,     "SD",     8192, NULL, 2, NULL, 0);

  Serial.println("Sistema CVT Imperador Iniciado!");
}

void loop() {
  // O loop fica livre. O RTOS gerencia as Tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
