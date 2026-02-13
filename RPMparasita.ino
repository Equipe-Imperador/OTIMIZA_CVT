#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// ========================== CONFIGURAÇÕES FÍSICAS ========================
#define PIN_RPM          33
#define PIN_VEL_TRAS     32 
#define PIN_VEL_DIANT_E  14
#define PIN_VEL_DIANT_D  27
#define SD_CS            5

// --- CONFIGURAÇÃO MECÂNICA ---
#define DENTES_RPM       1   
#define DENTES_TRAS      3   
#define DENTES_DIANT     5   
#define REDUCAO_TRAS     9.0f
#define DIAMETRO_TRAS    0.54f
#define DIAMETRO_DIANT   0.52f

const float CIRCUM_TRAS  = 3.14159f * DIAMETRO_TRAS;
const float CIRCUM_DIANT = 3.14159f * DIAMETRO_DIANT;

// --- AJUSTES DE FILTRO (DEBOUNCE) ---
const unsigned long DEBOUNCE_RPM   = 5000;  
const unsigned long DEBOUNCE_TRAS  = 3000;  
const unsigned long DEBOUNCE_DIANT = 5000;  

// --- AJUSTES DE TIMEOUT INDEPENDENTES (em microssegundos) ---
const unsigned long TIMEOUT_RPM_US   = 1000000; // 1 segundo
const unsigned long TIMEOUT_TRAS_US  = 5000000; // Tempo para zerar traseira
const unsigned long TIMEOUT_DIANT_US = 1200000; // Tempo para zerar dianteira

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
  if (diff < DEBOUNCE_TRAS) return;
  v_dtTras = diff;
  v_lastTras = agora;
}

void IRAM_ATTR isrDiantE() {
  unsigned long agora = micros();
  unsigned long diff = agora - v_lastDiantE;
  if (diff < DEBOUNCE_DIANT) return;
  v_dtDiantE = diff;
  v_lastDiantE = agora;
}

void IRAM_ATTR isrDiantD() {
  unsigned long agora = micros();
  unsigned long diff = agora - v_lastDiantD;
  if (diff < DEBOUNCE_DIANT) return;
  v_dtDiantD = diff;
  v_lastDiantD = agora;
}

// ========================== TAREFAS RTOS =================================
void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  static float last_valid_rpm = 0.0f;
  static float last_valid_vTras = 0.0f;
  static float last_valid_vDiantE = 0.0f;
  static float last_valid_vDiantD = 0.0f;

  for (;;) {
    SpeedPacket packet;
    unsigned long agora_us = micros();
    unsigned long dR, dT, dDE, dDD;
    unsigned long lR, lT, lDE, lDD;

    noInterrupts();
    dR = v_dtRPM;    lR = v_lastRPM;
    dT = v_dtTras;   lT = v_lastTras;
    dDE = v_dtDiantE; lDE = v_lastDiantE;
    dDD = v_dtDiantD; lDD = v_lastDiantD;
    interrupts();

    packet.timestamp = millis();
    
    // 1. Cálculo Bruto com Timeouts Independentes
    float raw_rpm     = (agora_us - lR > TIMEOUT_RPM_US || dR == 0) ? 0.0f : (60000000.0f / (dR * DENTES_RPM));
    float raw_vTras   = (agora_us - lT > TIMEOUT_TRAS_US || dT == 0) ? 0.0f : ((1000000.0f * CIRCUM_TRAS * 3.6f) / (dT * REDUCAO_TRAS * DENTES_TRAS));
    float raw_vDiantE = (agora_us - lDE > TIMEOUT_DIANT_US || dDE == 0) ? 0.0f : ((1000000.0f * CIRCUM_DIANT * 3.6f) / (dDE * DENTES_DIANT));
    float raw_vDiantD = (agora_us - lDD > TIMEOUT_DIANT_US || dDD == 0) ? 0.0f : ((1000000.0f * CIRCUM_DIANT * 3.6f) / (dDD * DENTES_DIANT));

    // 2. Filtro de Plausibilidade (mantido conforme original)
    if (raw_rpm > 0 && last_valid_rpm > 0 && fabs(raw_rpm - last_valid_rpm) > 1500.0f) raw_rpm = last_valid_rpm;
    if (raw_vTras > 0 && last_valid_vTras > 0 && fabs(raw_vTras - last_valid_vTras) > 15.0f) raw_vTras = last_valid_vTras;
    if (raw_vDiantE > 0 && last_valid_vDiantE > 0 && fabs(raw_vDiantE - last_valid_vDiantE) > 15.0f) raw_vDiantE = last_valid_vDiantE;
    if (raw_vDiantD > 0 && last_valid_vDiantD > 0 && fabs(raw_vDiantD - last_valid_vDiantD) > 15.0f) raw_vDiantD = last_valid_vDiantD;

    last_valid_rpm = raw_rpm;
    last_valid_vTras = raw_vTras;
    last_valid_vDiantE = raw_vDiantE;
    last_valid_vDiantD = raw_vDiantD;

    packet.rpm = raw_rpm;
    packet.vTras = raw_vTras;
    packet.vDiantE = raw_vDiantE;
    packet.vDiantD = raw_vDiantD;

    xQueueSend(dataQueue, &packet, 0);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); 
  }
}

void TaskSD(void *pvParameters) {
  SpeedPacket p;
  int counter = 0;
  for (;;) {
    if (xQueueReceive(dataQueue, &p, portMAX_DELAY) == pdPASS) {
      if (dataFile) {
        dataFile.printf("%u;%.0f;%.1f;%.1f;%.1f\n", p.timestamp, p.rpm, p.vTras, p.vDiantE, p.vDiantD);
        if (++counter >= 50) {
          dataFile.flush();
          counter = 0;
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RPM, INPUT_PULLUP);
  pinMode(PIN_VEL_TRAS, INPUT_PULLUP);
  pinMode(PIN_VEL_DIANT_E, INPUT);
  pinMode(PIN_VEL_DIANT_D, INPUT);

  if (!SD.begin(SD_CS)) {
    Serial.println("ERRO SD!");
  } else {
    char name[20];
    int n = 1;
    while (n < 1000) {
      sprintf(name, "/cvt_%d.csv", n);
      if (!SD.exists(name)) break;
      n++;
    }
    dataFile = SD.open(name, FILE_WRITE);
    if (dataFile) {
      dataFile.println("time_ms;rpm;vel_tras;vel_diant_e;vel_diant_d");
      dataFile.flush();
    }
  }

  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_TRAS), isrTras, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_E), isrDiantE, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_D), isrDiantD, FALLING);

  dataQueue = xQueueCreate(200, sizeof(SpeedPacket));
  xTaskCreatePinnedToCore(TaskSensor, "Sensor", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskSD, "SD", 8192, NULL, 2, NULL, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
