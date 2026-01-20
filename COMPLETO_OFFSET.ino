#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// =================================================================
// ========================== CONFIGURAÇÕES ========================
// =================================================================

// Pinos dos Sensores
#define PIN_RPM          35 // Sensor de RPM (Motor/Eixo 1)
#define PIN_VEL_TRAS     32 // Sensor de Velocidade Traseira
#define PIN_VEL_DIANT_E  33
//#define PIN_VEL_DIANT_D  25
#define SD_CS            5

// --- AJUSTES DE OFFSET (CALIBRAÇÃO) ---
float OFFSET_RPM     = 1.0f;
float OFFSET_TRAS    = 1.0f; 
float OFFSET_DIANT_E = 1.0f;
//float OFFSET_DIANT_D = 1.0f;

// --- CONFIGURAÇÃO MECÂNICA ---
#define DIAMETRO_TRAS    0.54f
#define DENTES_TRAS      3
#define REDUCAO_TRAS     9.5f
const float CIRCUM_TRAS = 3.14159f * DIAMETRO_TRAS;

#define DIAMETRO_DIANT   0.52f
#define DENTES_DIANT     1
const float CIRCUM_DIANT = 3.14159f * DIAMETRO_DIANT;

#define DENTES_RPM       1  // Pulsos por volta do motor

// Parâmetros de Filtro
const unsigned long TIMEOUT_US = 500000;    
const unsigned long DEBOUNCE_RPM = 11765;    // Debounce específico para motor
const unsigned long DEBOUNCE_VEL = 1000;

// Estrutura de Dados para a Fila e SD
struct SpeedPacket {
  uint32_t timestamp;
  float rpm;
  float vTras;
  float vDiantE;
  //float vDiantD;
};

// Variáveis Globais
QueueHandle_t dataQueue;
File dataFile;

// Variáveis de Interrupção (Voláteis)
volatile unsigned long dtRPM = 0, lastRPM = 0;
volatile unsigned long dtTras = 0, lastTras = 0;
volatile unsigned long dtDiantE = 0, lastDiantE = 0;
//volatile unsigned long dtDiantD = 0, lastDiantD = 0;

// =================================================================
// ======================== INTERRUPÇÕES ===========================
// =================================================================

void IRAM_ATTR isrRPM() {
  unsigned long agora = micros();
  if (agora - lastRPM < DEBOUNCE_RPM) return;
  dtRPM = agora - lastRPM;
  lastRPM = agora;
}

void IRAM_ATTR isrTras() {
  unsigned long agora = micros();
  if (agora - lastTras < DEBOUNCE_VEL) return;
  dtTras = agora - lastTras;
  lastTras = agora;
}

void IRAM_ATTR isrDiantE() {
  unsigned long agora = micros();
  if (agora - lastDiantE < DEBOUNCE_VEL) return;
  dtDiantE = agora - lastDiantE;
  lastDiantE = agora;
}

/*void IRAM_ATTR isrDiantD() {
  unsigned long agora = micros();
  if (agora - lastDiantD < DEBOUNCE_VEL) return;
  dtDiantD = agora - lastDiantD;
  lastDiantD = agora;
}*/

// =================================================================
// ====================== FUNÇÕES DE CÁLCULO =======================
// =================================================================

float calcRPM(unsigned long dt) {
  if (dt == 0) return 0.0f;
  return (60000000.0f / (dt * DENTES_RPM)) * OFFSET_RPM;
}

float calcVelTras(unsigned long dt) {
  if (dt == 0) return 0.0f;
  float vel = (1000000.0f * CIRCUM_TRAS * 3.6f) / (dt * REDUCAO_TRAS * DENTES_TRAS);
  return vel * OFFSET_TRAS;
}

float calcVelDiant(unsigned long dt, float offset) {
  if (dt == 0) return 0.0f;
  float vel = (1000000.0f * CIRCUM_DIANT * 3.6f) / (dt * DENTES_DIANT);
  return vel * offset;
}

// =================================================================
// ========================== TAREFAS RTOS =========================
// =================================================================

void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz

  for (;;) {
    SpeedPacket packet;
    unsigned long _dtR, _dtT, _dtDE, _dtDD;
    unsigned long _lastR, _lastT, _lastDE/*, _lastDD*/;
    unsigned long agora_us = micros();

    noInterrupts();
    _dtR = dtRPM;      _lastR = lastRPM;
    _dtT = dtTras;     _lastT = lastTras;
    _dtDE = dtDiantE;  _lastDE = lastDiantE;
   // _dtDD = dtDiantD;  _lastDD = lastDiantD;
    interrupts();

    packet.timestamp = millis();
    packet.rpm    = (agora_us - _lastR > TIMEOUT_US) ? 0.0f : calcRPM(_dtR);
    packet.vTras  = (agora_us - _lastT > TIMEOUT_US) ? 0.0f : calcVelTras(_dtT);
    packet.vDiantE = (agora_us - _lastDE > TIMEOUT_US) ? 0.0f : calcVelDiant(_dtDE, OFFSET_DIANT_E);
    //packet.vDiantD = (agora_us - _lastDD > TIMEOUT_US) ? 0.0f : calcVelDiant(_dtDD, OFFSET_DIANT_D);

    xQueueSend(dataQueue, &packet, 0);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskSD(void *pvParameters) {
  SpeedPacket receivedPacket;
  int counter = 0;

  for (;;) {
    if (xQueueReceive(dataQueue, &receivedPacket, portMAX_DELAY) == pdPASS) {
      if (dataFile) {
        // Formato: timestamp; RPM; VelTras; VelDiantE; VelDiantD
        dataFile.printf("%u;%.0f;%.2f;%.2f\n", 
                        receivedPacket.timestamp, 
                        receivedPacket.rpm,
                        receivedPacket.vTras, 
                        receivedPacket.vDiantE, 
                        //receivedPacket.vDiantD
          );
        
        if (++counter >= 100) {
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

  pinMode(PIN_RPM, INPUT);
  pinMode(PIN_VEL_TRAS, INPUT);
  pinMode(PIN_VEL_DIANT_E, INPUT);
  //pinMode(PIN_VEL_DIANT_D, INPUT_PULLUP);

  if (!SD.begin(SD_CS)) {
    Serial.println("Erro SD!");
    while(1);
  }
  
  dataFile = SD.open("/telemetria.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("timestamp_ms;rpm;vTras;vDiantE");
  }

  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_TRAS), isrTras, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_E), isrDiantE, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_VEL_DIANT_D), isrDiantD, FALLING);

  dataQueue = xQueueCreate(100, sizeof(SpeedPacket));

  if (dataQueue != NULL) {
    xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskSD, "TaskSD", 4096, NULL, 2, NULL, 0);
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
