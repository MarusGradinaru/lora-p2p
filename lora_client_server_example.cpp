#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

//#define TX_MODULE      // comment for RX_MODULE
#define DEBUG_MODE     // comment for release mode
 
// Module pin configuration
#define PIN_CS     0
#define PIN_CLK    0
#define PIN_MOSI   0
#define PIN_MISO   0
#define PIN_RESET  0
#define PIN_BUSY   0
#define PIN_RX_EN  0
#define PIN_TX_EN  0
#define PIN_DIO1   0

// Touch buttons settings
#define PIN_CMD1 T0  // GPIO4
#define PIN_CMD2 T2  // GPIO2
#define TOUCH_THRESHOLD 15 
#define RELEASE_THRESHOLD 40
#define DEBOUNCE_TIME 500

// Server commands
#define CMD_1  0xC1E8
#define CMD_2  0x3A7B

#ifdef DEBUG_MODE
  #define Print(msg)          Serial.print(msg)
  #define PrintLn(msg)        Serial.println(msg)
  #define SafePrint(msg)      _SafePrint(msg)
  #define SafePrintLn(msg)    _SafePrintLn(msg)
  #define SafePrintBuff(msg, buff, len)  _SafePrintBuff(msg, buff, len)
  SemaphoreHandle_t serialMutex;
#else
  #define Print(msg)          // remove it
  #define PrintLn(msg)        // remove it
  #define SafePrint(msg)      // remove it
  #define SafePrintLn(msg)    // remove it
  #define SafePrintBuff(msg, buff, len)  // remove it
#endif

void _SafePrint(const __FlashStringHelper* msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.print(msg);
  xSemaphoreGive(serialMutex);
}

void _SafePrintLn(const __FlashStringHelper* msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println(msg);
  xSemaphoreGive(serialMutex);
}

void _SafePrintBuff(const __FlashStringHelper* msg, uint8_t* buffer, uint16_t len) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.print(F("[SYS] ")); Serial.print(msg); Serial.print(" = ");
  for (int i = 2; i < len+2; i++) {
    if (buffer[i] < 0x10) { Serial.print("0"); }
    Serial.print(buffer[i], HEX);
    if (i < len+1) { Serial.print(", "); }
  }
  Serial.println();
  xSemaphoreGive(serialMutex);  
}

bool CheckLoraResult(int16_t state, bool errStop = false) {
  if (state == RADIOLIB_ERR_NONE) {
    SafePrintLn(F("done !"));
    return true;
  } else {
    #if defined(DEBUG_MODE)
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.print(F("failed, code "));
      Serial.println(state);
      xSemaphoreGive(serialMutex);
    #endif
    if (errStop) { while (true) { delay(1000); } }
    return false;
  }
} 

#if defined(TX_MODULE) //-------------------------- TX MODULE -------------------------

volatile bool touchEnabled = false;
const uint8_t pinBtn1 = PIN_CMD1;
const uint8_t pinBtn2 = PIN_CMD2;
QueueHandle_t touchQueue;

void IRAM_ATTR touchISR1() {
  if (touchEnabled) {
    touchEnabled = false;
    xQueueSendFromISR(touchQueue, &pinBtn1, NULL);
  }
}

void IRAM_ATTR touchISR2() {
  if (touchEnabled) {
    touchEnabled = false;
    xQueueSendFromISR(touchQueue, &pinBtn2, NULL);  
  }
}

void touchTask(void* pvParameters) {
  uint8_t pinBtn;
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t data[12]; size_t size;
  bool doTX; String cmdName;

  while (true) {
    if (xQueueReceive(touchQueue, &pinBtn, portMAX_DELAY) == pdPASS) {
      uint32_t start_ms = millis();
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.print(F("Button at GPIO")); Serial.print(pinBtn); Serial.println(F(" touched !"));
      xSemaphoreGive(serialMutex);

      memset(data, 0x00, sizeof(data)); doTX = false;
      switch (pinBtn) {
        case PIN_CMD1:
          size = 3; cmdName = "Command 1"; doTX = true;
          data[0] = CMD_1 & 0xFF; data[1] = (CMD_1 >> 8) & 0xFF;  // Command ID (LSB + MSB)
          data[2] = 0xA1; data[3] = 0xB2; data[4] = 0xC3;  // Command data...
          break;          
        case PIN_CMD2:
          size = 5; cmdName = "Command 2"; doTX = true;
          data[0] = CMD_2 & 0xFF; data[1] = (CMD_2 >> 8) & 0xFF;  // Command ID (LSB + MSB)
          data[2] = 0x33; data[3] = 0x55; data[4] = 0x77; data[5] = 0x99; data[6] = 0xBB;  // Command data...
          break; 
      }

      if (doTX) {
        #if defined(DEBUG_MODE)
          xSemaphoreTake(serialMutex, portMAX_DELAY);
          Serial.print(F("[SX1262] Sending ")); Serial.println(cmdName+"...");
          xSemaphoreGive(serialMutex);
        #endif  
        SafePrintBuff(F("Command Data"), data, size);
        state = LORA.transmit(data, size+2);
        if (CheckLoraResult(state)) {
          memset(data, 0x00, sizeof(data));
          SafePrint(F("[SX1262] Waiting for replay... "));
          state = LORA.receive(data, sizeof(data));
          if (CheckLoraResult(state)) { SafePrintBuff(F("Received Data"), data, 10); }
        }
        SafePrintLn(F("[SX1262] Back to standby."));
        LORA.standby();
      }

      int stable_count = 0;
      while (stable_count < 5) {
        if (touchRead(pinBtn) >= RELEASE_THRESHOLD) { stable_count++; } else { stable_count = 0; }
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      uint32_t elapsed = millis() - start_ms;
      if (elapsed < DEBOUNCE_TIME) { vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME - elapsed)); }
      touchEnabled = true; 
    }
  }
}  

#else  //------------------------------------------ RX MODULE -------------------------

TaskHandle_t hLoraServer = NULL;

void IRAM_ATTR IrqDio1(void) { 
  vTaskNotifyGiveFromISR(hLoraServer, NULL); 
}

void LoraServerTask(void* pvParameters) {
  bool doTX;
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t data[12]; 

  while (true) {
    // wait for RF command (DIO1 interrupt)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
    SafePrintLn(F("[SYS] New packet received."));

    // check for valid packet length
    size_t size = LORA.getPacketLength();
    if (size < 2 || size > 12) {
      LORA.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
      #if defined(DEBUG_MODE)
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print(F("[SYS] Invalid packet size: "));
        Serial.print(size); Serial.println(F(" bytes. Ignoring packet !"));
        xSemaphoreGive(serialMutex);
      #endif
      continue;
    }

    // read the packet from SX1262 device
    memset(data, 0x00, sizeof(data));
    state = LORA.readData(data, size);
    if (state != RADIOLIB_ERR_NONE) {
      LORA.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
      #if defined(DEBUG_MODE)
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print(F("[SX1262] Failed to read packet. Error code: "));
        Serial.println(state);
        xSemaphoreGive(serialMutex);
      #endif  
      continue;
    }  
    SafePrintLn(F("[SX1262] The packet has been successfully received."));
    size -= 2;

    // handle the requested command
    uint16_t CMD = data[0] | (data[1] << 8); doTX = false;
    switch (CMD) {

      case CMD_1: 
        SafePrintLn(F("[SYS] Client command: CMD 1"));
        SafePrintBuff(F("Command Data"), data, size);
        LORA.clearDio1Action(); doTX = true; size = 7;
        for (int i = 0; i < size; i++) { data[i + 2] = i + 1; }
        break;

      case CMD_2:
        SafePrintLn(F("[SYS] Client command: CMD 2"));
        SafePrintBuff(F("Command Data"), data, size);
        LORA.clearDio1Action(); doTX = true; size = 10;
        for (int i = 0; i < size; i++) { data[i + 2] = size - i; }
        break;

      default:
        SafePrintLn(F("[SYS] Unknown command."));
    }

    if (doTX) {
      SafePrintLn(F("[SX1262] Sending requested data... "));
      SafePrintBuff(F("Replay Data"), data, size);
      state = LORA.transmit(data, size+2);
      CheckLoraResult(state);
      LORA.setDio1Action(IrqDio1);
      SafePrint(F("[SX1262] Back to listening mode... "));
      state = LORA.startReceive();
      CheckLoraResult(state);
    }
  }
}

#endif //-------------------------------------------------------------------------------


SPISettings mySPISettings(2000000, MSBFIRST, SPI_MODE0);  // 10 cm long wires SPI
SX1262 LORA = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);

void setup() {
  #if defined(DEBUG_MODE)
    Serial.begin(115200); delay(1000);
    serialMutex = xSemaphoreCreateMutex();
  #endif

  #if defined(TX_MODULE)
    PrintLn(F("Starting program for TX Module (Client)... "));
  #else  
    PrintLn(F("Starting program for RX Module (Server)... "));
  #endif

  SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Print(F("[SX1262] Initializing LoRa... "));
  int state = LORA.begin(
    869.5, // Frequency: P band, 500mW (27dBm), 10% (6s/min) [863.0 MHz + 1/2 BW .. 870.0 MHz - 1/2 BW] 
    125.0, // Bandwidth (BW) [7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500] KHz
    9,     // Spreading Factor (SF) [5..12]
    7,     // Coding Rate: 4/7 (CR) [5..8]
    0xE3,  // Sync Word: Private [0x12, 0x34] 
    9,     // TX Power (dBm) [-9..22] / with 5 dBi antenna = 14 dBm (25 mW)
    8,     // Preamble Length [6..20] 
    1.7,   // TCXO Voltage (V) [1.6, 1.7, 1.8, 2.2, 2.4, 2.7, 3.0, 3.3]
    false  // Use LDO ? [false: DC-DC, true: LDO]
  );
  CheckLoraResult(state, true);

  LORA.setRfSwitchPins(PIN_RX_EN, PIN_TX_EN);

  #if defined(TX_MODULE)
    Print(F("[SYS] Starting Touch Buttons task... "));
    touchQueue = xQueueCreate(5, sizeof(uint8_t));
    xTaskCreatePinnedToCore(touchTask, "TouchTask", 4096, NULL, 1, NULL, 1);
    touchAttachInterrupt(PIN_CMD1, touchISR1, TOUCH_THRESHOLD); 
    touchAttachInterrupt(PIN_CMD2, touchISR2, TOUCH_THRESHOLD); 
    PrintLn(F("done !")); 
    Print(F("[SX1262] Entering standby mode... "));
    CheckLoraResult(LORA.standby(), true);
    touchEnabled = true;
  #else
    LORA.setDio1Action(IrqDio1); 
    Print(F("[SYS] Starting LoRa Server task... "));
    if (xTaskCreatePinnedToCore(LoraServerTask, "LoraServerTask", 4096, NULL, 1, &hLoraServer, 1) == pdPASS) 
      { SafePrintLn(F("done !")); } else { SafePrintLn(F("failed !")); while (true) { delay(1000); } }
    SafePrint(F("[SX1262] Starting to listen... "));
    CheckLoraResult(LORA.startReceive(), true);
  #endif
}

void loop() {
  delay(1000);
}
