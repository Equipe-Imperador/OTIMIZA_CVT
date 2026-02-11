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
#define DENTES_RPM       1   
#define DENTES_TRAS      3   
#define DENTES_DIANT     5   // Verifique se são 5 imãs mesmo na frente
#define REDUCAO_TRAS     9.0f
#define DIAMETRO_TRAS    0.54f
#define DIAMETRO_DIANT   0.52f

const float CIRCUM_TRAS  = 3.14159f * DIAMETRO_TRAS;
const float CIRCUM_DIANT = 3.14159f * DIAMETRO_DIANT;

// --- AJUSTES ---
const unsigned long DEBOUNCE_RPM = 10000;   // 10ms (~6000 RPM max)
const unsigned long DEBOUNCE_VEL = 2000;    // 2ms
const unsigned long TIMEOUT_US   = 1000000; // Aumentei para 1s para evitar zeros falsos em baixa

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

// Variáveis de Interrupção
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

void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    SpeedPacket packet;
    unsigned long agora_us = micros();
    unsigned long dR, dT, dDE, dDD;
    unsigned long lR, lT, lDE, lDD;

    // Apenas COPIAMOS os valores. NÃO zeramos v_dt.
    noInterrupts();
    dR = v_dtRPM;    lR = v_lastRPM;
    dT = v_dtTras;   lT = v_lastTras;
    dDE = v_dtDiantE; lDE = v_lastDiantE;
    dDD = v_dtDiantD; lDD = v_lastDiantD;
    interrupts();

    packet.timestamp = millis();
    
    // Lógica de Timeout: Se o último pulso foi há mais de TIMEOUT_US, a velocidade é 0.
    packet.rpm     = (agora_us - lR > TIMEOUT_US) ? 0.0f : (60000000.0f / (dR * DENTES_RPM));
    packet.vTras   = (agora_us - lT > TIMEOUT_US) ? 0.0f : ((1000000.0f * CIRCUM_TRAS * 3.6f) / (dT * REDUCAO_TRAS * DENTES_TRAS));
    packet.vDiantE = (agora_us - lDE > TIMEOUT_US) ? 0.0f : ((1000000.0f * CIRCUM_DIANT * 3.6f) / (dDE * DENTES_DIANT));
    packet.vDiantD = (agora_us - lDD > TIMEOUT_US) ? 0.0f : ((1000000.0f * CIRCUM_DIANT * 3.6f) / (dDD * DENTES_DIANT));

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

  // IMPORTANTE: Se os sensores forem do tipo efeito Hall que chaveiam GND, 
  // o INPUT_PULLUP é obrigatório para o sinal não flutuar.
  pinMode(PIN_RPM,INPUT_PULLUP );
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
