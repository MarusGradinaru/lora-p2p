#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#ifdef PICO_BOARD
  #include <FreeRTOS.h>
  #include <semphr.h>
#endif

#define TX_MODULE    // comment for RX_MODULE
#define DEBUG_MODE   // comment for release mode

// Module pin configuration (GPIO)
#ifdef ESP32_BOARD
  #define PIN_CS     5
  #define PIN_CLK    18
  #define PIN_MOSI   23
  #define PIN_MISO   19
  #define PIN_RESET  27
  #define PIN_BUSY   26
  #define PIN_RX_EN  25
  #define PIN_TX_EN  33
  #define PIN_DIO1   32
#else
  #define PIN_CS     17 
  #define PIN_CLK    18
  #define PIN_MOSI   19  // TX
  #define PIN_MISO   16  // RX
  #define PIN_RESET  15
  #define PIN_BUSY   14
  #define PIN_RX_EN  21 
  #define PIN_TX_EN  20
  #define PIN_DIO1   13
#endif

// Module settings
#define FREQ       869.5   // Frequency: P band, 500mW (27dBm), 10% [863.0 MHz + 1/2 BW .. 870.0 MHz - 1/2 BW] 
#define BW         125.0   // Bandwidth [7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500] KHz
#define SF         9       // Spreading Factor [5..12]
#define CR         7       // Coding Rate: 4/7 [5..8]
#define SYNCW      0xE3    // Sync Word: Custom [Private: 0x12, Public: 0x34] 
#define TX_PWR     9       // TX Power (dBm) [-9..22] / with 5 dBi antenna = 14 dBm (25 mW)
#define TX_DC      10.0    // TX Duty Cycle (% percent)
#define PAMB       8       // Preamble Length [6..20] 
#define XOV        1.7     // TCXO Voltage (V) [1.6, 1.7, 1.8, 2.2, 2.4, 2.7, 3.0, 3.3]
#define LDO        false   // Use LDO only ? [false:LDO and DC-DC, true: just LDO]

// Touch buttons settings
#define PIN_CMD1   T0  // GPIO4
#define PIN_CMD2   T2  // GPIO2
#define TOUCH_THRESHOLD   15 
#define RELEASE_THRESHOLD 40
#define DEBOUNCE_TIME     500

// Server commands
#define CMD_1      0xC1E8
#define CMD_2      0x3A7B


// ------ Global objects -----------------------------------

SPISettings mySPISettings(2000000, MSBFIRST, SPI_MODE0);  // 10 cm long wires SPI
SX1262 LORA = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);

// ------ Debug functions ----------------------------------

#ifdef DEBUG_MODE

  #define SerialTake()                xSemaphoreTakeRecursive(serialMutex, portMAX_DELAY)
  #define SerialGive()                xSemaphoreGiveRecursive(serialMutex);
  #define Print(msg)                  Serial.print(msg)
  #define PrintLn(msg)                Serial.println(msg)
  #define SafePrintLn(msg)            _SafePrintLn(msg)
  #define PrintStat()                 _PrintStat()
  #define PrintTime(msg, wt, pt)      _PrintTime(F(msg), wt, pt) 
  #define PrintBuff(msg, buff, len)   _PrintBuff(F(msg), buff, len)
  
SemaphoreHandle_t serialMutex;

void _SafePrintLn(const __FlashStringHelper* msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println(msg);
  xSemaphoreGive(serialMutex);
}

void _PrintStat() {
  Serial.print(F("[SX1262] Status:  RSSI = ")); Serial.print(LORA.getRSSI()); 
  Serial.print(F(" dBm,  SNR = ")); Serial.print(LORA.getSNR());
  Serial.print(F(" dB,  Freq. Error = ")); Serial.print(LORA.getFrequencyError());
  Serial.println(F(" Hz"));
}

void _PrintTime(const __FlashStringHelper* msg, uint32_t work_time, uint32_t pause_time) {
  Serial.print(F("[SYSTEM] ")); Serial.print(msg); 
  Serial.print(" = "); Serial.print(work_time);
  if (pause_time == 0) Serial.println(" ms");
  else { Serial.print(F(" ms,  Wait Time = ")); Serial.print(pause_time); Serial.println(" ms"); }
}

void _PrintBuff(const __FlashStringHelper* msg, uint8_t* buffer, uint16_t len) {
  Serial.print(F("[SYSTEM] ")); Serial.print(msg); Serial.print(" = ");
  for (int i = 2; i < len+2; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    if (i < len+1) Serial.print(", ");
  }
  Serial.println();
}

#else  // RELEASE MODE

  #define SerialTake()                    ((void)0)
  #define SerialGive()                    ((void)0)
  #define Print(msg)                      ((void)0)
  #define PrintLn(msg)                    ((void)0)
  #define SafePrintLn(msg)                ((void)0)
  #define PrintStat()                     ((void)0)
  #define PrintTime(msg, wt, pt)          ((void)0)
  #define PrintBuff(msg, buff, len)       ((void)0)

#endif

// ------ Common functions ----------------------------------

bool CheckLoraResult(int16_t state, bool errStop = false) {
  if (state == RADIOLIB_ERR_NONE) {
    PrintLn(F("done !"));
    return true;
  } else {
    Print(F("failed, code ")); PrintLn(state);
    if (errStop) { while (true) delay(1000); }
    return false;
  }
} 

bool ReadyForTX(uint32_t deadline, uint32_t& wait_time) {
  int32_t delta = (int32_t)(millis() - deadline);
  if (delta >= 0) { wait_time = 0; return true; } 
   else {wait_time = (uint32_t)(-delta); return false; }
}

#ifdef TX_MODULE //-------------------------------- TX MODULE -------------------------

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

void TouchTask(void* pvParameters) {
  uint8_t pinBtn;
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t data[12]; size_t size;
  uint32_t next_tx = 0;
  float ps_const = (100.0 / TX_DC) - 1.0;
  const __FlashStringHelper* cmdName;
  bool doTX;

  while (true) {
    if (xQueueReceive(touchQueue, &pinBtn, portMAX_DELAY) == pdPASS) {
      uint32_t start_ms = millis();
      SerialTake();
      Print(F("[SYSTEM] Button at GPIO")); Print(pinBtn); PrintLn(F(" touched !"));

      memset(data, 0x00, sizeof(data)); doTX = false;
      switch (pinBtn) {
        case PIN_CMD1:
          size = 3; cmdName = F("Command 1 and Data"); doTX = true;
          data[0] = CMD_1 & 0xFF; data[1] = (CMD_1 >> 8) & 0xFF;  // Command ID (LSB + MSB)
          data[2] = 0xA1; data[3] = 0xB2; data[4] = 0xC3;  // Command data...
          break;          
        case PIN_CMD2:
          size = 5; cmdName = F("Command 2 and Data"); doTX = true;
          data[0] = CMD_2 & 0xFF; data[1] = (CMD_2 >> 8) & 0xFF;  // Command ID (LSB + MSB)
          data[2] = 0x33; data[3] = 0x55; data[4] = 0x77; data[5] = 0x99; data[6] = 0xBB;  // Command data...
          break; 
      }

      if (doTX) {
        uint32_t wtime;
        if (!ReadyForTX(next_tx, wtime)) { 
          Print(F("[SYSTEM] Please wait ")); Print(wtime); PrintLn(" ms");
        } else {
          PrintBuff("Data Ready", data, size);
          Print(F("[SX1262] Sending ")); Print(cmdName); Print("...");
          uint32_t tx_start = millis();
          state = LORA.transmit(data, size+2);
          if (CheckLoraResult(state)) {
            uint32_t tx_time = millis() - tx_start;
            uint32_t ps_time = ceil(tx_time * ps_const);
            next_tx = tx_start + tx_time + ps_time;
            PrintTime("TX Time", tx_time, ps_time);
            memset(data, 0x00, sizeof(data));
            Print(F("[SX1262] Waiting for replay... "));
            state = LORA.receive(data, sizeof(data));
            if (CheckLoraResult(state)) { 
              PrintBuff("Received Data", data, 10); 
              PrintStat();
            }
          }
          PrintLn(F("[SX1262] Back to standby."));
          LORA.standby();
        }
      }

      PrintLn(""); SerialGive();

      int stable_count = 0;
      while (stable_count < 5) {
        if (touchRead(pinBtn) >= RELEASE_THRESHOLD) stable_count++; else  stable_count = 0; 
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      uint32_t elapsed = millis() - start_ms;
      if (elapsed < DEBOUNCE_TIME) vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME - elapsed)); 
      touchEnabled = true; 
    }
  }
}  

#else  //------------------------------------------ RX MODULE -------------------------

#ifdef ESP32_BOARD
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif

TaskHandle_t hLoraServer = NULL;

void ISR_ATTR IrqDio1(void) { 
  vTaskNotifyGiveFromISR(hLoraServer, NULL); 
}

void LoraServerTask(void* pvParameters) {
  uint8_t data[12]; 
  bool doTX;

  while (true) {
    // wait for RF command (DIO1 interrupt)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    SerialTake(); 
    PrintLn(F("[SYSTEM] New packet detected."));

    // check for valid packet length
    size_t size = LORA.getPacketLength();
    if (size < 2 || size > 12) {
      LORA.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
      Print(F("[SYSTEM] Invalid packet size: ")); Print(size); 
      PrintLn(F(" bytes. Ignoring packet !"));
      SerialGive(); continue;
    }

    // read the packet from SX1262 device
    memset(data, 0x00, sizeof(data));
    int16_t state = LORA.readData(data, size);
    if (state != RADIOLIB_ERR_NONE) {
      LORA.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
      Print(F("[SX1262] Failed to read packet. Error code: ")); PrintLn(state);
      SerialGive(); continue;
    }  
    PrintLn(F("[SX1262] The packet has been successfully received."));
    PrintStat(); size -= 2;

    // handle the requested command
    uint16_t CMD = data[0] | (data[1] << 8); doTX = false;
    switch (CMD) {

      case CMD_1: 
        PrintLn(F("[SYSTEM] Client command: CMD 1"));
        PrintBuff("Command Data", data, size);
        LORA.clearDio1Action(); doTX = true; size = 7;
        for (int i = 0; i < size; i++) { data[i + 2] = i + 1; }
        break;

      case CMD_2:
        PrintLn(F("[SYSTEM] Client command: CMD 2"));
        PrintBuff("Command Data", data, size);
        LORA.clearDio1Action(); doTX = true; size = 10;
        for (int i = 0; i < size; i++) { data[i + 2] = size - i; }
        break;

      default:
        PrintLn(F("[SYSTEM] Unknown command."));
    }

    if (doTX) {
      PrintBuff("Replay Data", data, size);
      PrintLn(F("[SX1262] Sending requested data... "));
      uint32_t tx_start = millis();
      CheckLoraResult(LORA.transmit(data, size+2));
      uint32_t tx_time = millis() - tx_start;
      PrintTime("TX Time", tx_time, 0);
      LORA.setDio1Action(IrqDio1);
      Print(F("[SX1262] Back to listening mode... "));
      CheckLoraResult(LORA.startReceive());
    }
    PrintLn(F("")); SerialGive();
  }
}

#endif //-------------------------------------------------------------------------------


void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(115200); delay(1500);
    serialMutex = xSemaphoreCreateRecursiveMutex();
  #endif

  #ifdef TX_MODULE
    PrintLn(F("[SYSTEM] Starting program for TX Module (Client)..."));
  #else  
    PrintLn(F("[SYSTEM] Starting program for RX Module (Server)..."));
  #endif

  #ifdef ESP32_BOARD 
    SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);
  #else
    SPI.setSCK(PIN_CLK);
    SPI.setRX(PIN_MISO);  // SPI0 RX (MISO)
    SPI.setTX(PIN_MOSI);  // SPIO TX (MOSI)
    SPI.begin();
  #endif  

  Print(F("[SX1262] Initializing LoRa... "));
  CheckLoraResult(LORA.begin(FREQ, BW, SF, CR, SYNCW, TX_PWR, PAMB, XOV, LDO), true);
  LORA.setRfSwitchPins(PIN_RX_EN, PIN_TX_EN);

  #if defined(TX_MODULE)  // ---------- TX MODULE SETUP --------------------------------

    SerialTake();
    Print(F("[SYSTEM] Starting Touch Buttons task... "));
    touchQueue = xQueueCreate(5, sizeof(uint8_t));
    if (xTaskCreatePinnedToCore(TouchTask, "TouchTask", 2048, NULL, 1, NULL, 1) == pdPASS) {
      touchAttachInterrupt(PIN_CMD1, touchISR1, TOUCH_THRESHOLD); 
      touchAttachInterrupt(PIN_CMD2, touchISR2, TOUCH_THRESHOLD); 
      PrintLn(F("done !"));
    } else {
      PrintLn(F("failed !"));
      while (true) delay(1000);  
    }  
    Print(F("[SX1262] Entering standby mode... "));
    CheckLoraResult(LORA.standby(), true);
    Print(""); SerialGive(); touchEnabled = true;

  #else                   // ---------- RX MODULE SETUP -------------------------------- 

    SerialTake();
    LORA.setDio1Action(IrqDio1); 
    Print(F("[SYSTEM] Starting LoRa Server task... "));

    #ifdef ESP32_BOARD
      if (xTaskCreatePinnedToCore(LoraServerTask, "LoraServerTask", 2048, NULL, 1, &hLoraServer, 1) == pdPASS) 
        { PrintLn(F("done !")); } else { PrintLn(F("failed !")); while (true) delay(1000); }
    #else
      if (xTaskCreateAffinitySet(LoraServerTask, "LoraServerTask", 2048, NULL, 1, 2, &hLoraServer) == pdPASS) 
        { PrintLn(F("done !")); } else { PrintLn(F("failed !")); while (true) delay(1000); }      
    #endif

    Print(F("[SX1262] Starting to listen... "));
    CheckLoraResult(LORA.startReceive(), true);
    Print(""); SerialGive();

  #endif                  // -----------------------------------------------------------
}

void loop() {
  delay(1000);
}
