#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include "person_sensor.h"

// === IR Configuration ===
static const int kNumCodes = 4;

// === Samsung Preset Codes ===
static const uint8_t kSamsungProtocol = NECX;
static const uint8_t kSamsungCodeBits = 32;

static const uint32_t kSamsungCodes[] = {
  0xE0E06798, // Power
  0xE0E0F00F, // Mute
  0xE0E0E21D, // Play
  0xE0E052AD, // Pause
};

typedef struct __attribute__ ((__packed__)) {
  uint8_t codeProtocols[kNumCodes];
  uint32_t codeValues[kNumCodes];
  uint8_t codeBits[kNumCodes];
} CodeConfig_t;
CodeConfig_t codeConfig = {};

void sendCode(int code_idx);

// === TFT Display Configuration ===
// Because of the limited number of pins available on the Circuit Playground Boards
// Software SPI is used
#define TFT_CS        0
#define TFT_RST       -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC        1
#define TFT_BACKLIGHT PIN_A3 // Display backlight pin
#if (SPI_INTERFACES_COUNT == 1)
  SPIClass* spi = &SPI;
#else
  SPIClass* spi = &SPI1;
#endif

#define TFT_MOSI      PIN_WIRE_SDA  // Data out
#define TFT_SCLK      PIN_WIRE_SCL  // Clock out
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// === Person Sensor Configuration ===
#define SOFT_I2C true
#define SOFT_SCL 9
#define SOFT_SDA 6
#define BAUD_RATE 9600
#define SAMPLE_PERIOD 200
static long readTimestamp;

SoftWire sw(SOFT_SDA, SOFT_SCL);

byte swTxBuffer[16];
byte swRxBuffer[sizeof(person_sensor_results_t)];
static int hardwareScan(void);
static int softwareScan(void);

void setup(){
  // Init Software I2C.
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setTimeout_ms(100);
  sw.begin();
  
  Serial.begin(BAUD_RATE);
  while(!Serial);
  Serial.println("Booting person TV remote");
  int nDevices = softwareScan();
  initScreen();
  
  // Use Samsung codes by default.
  memcpy(&codeConfig.codeValues, kSamsungCodes, sizeof(uint32_t) * kNumCodes);
  memset(&codeConfig.codeProtocols, kSamsungProtocol, sizeof(uint8_t) * kNumCodes);
  memset(&codeConfig.codeBits, kSamsungCodeBits, sizeof(uint8_t) * kNumCodes);
}

void loop(){
  person_sensor_results_t results;
  if(millis() - readTimestamp >= SAMPLE_PERIOD){
    person_sensor_results_t* results_ptr = &results;
    int8_t* results_bytes = (int8_t*) results_ptr;;
    int numBytes = sw.requestFrom(PERSON_SENSOR_I2C_ADDRESS, sizeof(person_sensor_results_t));
    for(int i = 0; i < sizeof(person_sensor_results_t); i++){
      results_bytes[i] = sw.read();
    }
      
    handleSensorResults(results);

    readTimestamp = millis();
  }
}

static int softwareScan(){
  int nDevices = 0;

  Serial.println("Scanning...");

  for(int address = 1; address < 127; address++){
    uint8_t startResult = sw.llStart((address << 1) + 1);
    sw.stop();

    if(startResult == 0){
      Serial.print("I2C device found at address 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
    }
  }

  if(nDevices == 0){
    Serial.println("No I2C devices found");
  }else{
    Serial.print(nDevices);
    Serial.print(" devices found at time ");
    Serial.println(millis());
  }

  return nDevices;
}

static void initScreen(){
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH); // Backlight on
  
  tft.init(240, 240);                // Init ST7789 240x240
  tft.setRotation(2);
  tft.setTextSize(2, 2);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextWrap(true);
  // large block of text
  tft.fillScreen(ST77XX_BLACK);
}

void sendCode(int code_idx) {
  CircuitPlayground.irSend.send(codeConfig.codeProtocols[code_idx], codeConfig.codeValues[code_idx], codeConfig.codeBits[code_idx]);
}

#define STATE_NO_FACE 0
#define STATE_FACE_ON 1
#define STATE_TURNED_AWAY 2
#define PAUSE_DELAY_MILLIS 4000
#define TURN_OFF_DELAY_MILLIS 10000

void displayState(int state, int millis_since_face, int millis_since_face_on) {
  Serial.println(state);

  tft.setCursor(0, 0);
  tft.fillRect(0, 0, 240, 80, ST77XX_BLACK);
  tft.setCursor(0, 0);

  char buf[100];
  switch(state) {
    case STATE_NO_FACE:
      tft.print("No Face\nTV OFF");
      break;
    case STATE_FACE_ON:
      sprintf(buf, "Face-on\npause in %ds.\nTV off in %ds",
                    (PAUSE_DELAY_MILLIS - millis_since_face_on) / 1000,
                    (TURN_OFF_DELAY_MILLIS - millis_since_face) / 1000);
      tft.print(buf);
      break;
    case STATE_TURNED_AWAY:
          sprintf(buf, "Faced away\nTV off in %ds",
                    (TURN_OFF_DELAY_MILLIS - millis_since_face) / 1000);
      tft.print(buf);
      break;
    default:
      Serial.println("Error - invalid state");
      break;
  }
}

void handleSensorResults(person_sensor_results_t results) {
  static int lastFaceSeenTime = 0;
  static int lastFaceOnTime = 0;
  static int state = STATE_NO_FACE;

  bool has_face = false;
  bool is_face_on = false;
  for (int i=0; i<results.num_faces; i++) {
    if (results.boxes[i].confidence > 90) {
      has_face = true;
      if (results.boxes[i].face_on) {
        is_face_on = true;
      }
    }
  }

  lastFaceSeenTime = has_face ? millis() : lastFaceSeenTime;
  lastFaceOnTime = is_face_on ? millis() : lastFaceOnTime;
  switch(state) {
    case STATE_NO_FACE:
      if (has_face) {
        sendCode(0); // Power on.
        lastFaceSeenTime = millis();
        if (is_face_on) {
          lastFaceOnTime = millis();
          state = STATE_FACE_ON;
          delay(100);
          sendCode(2); // Play.
        } else {
          state = STATE_TURNED_AWAY;
        }
      }
      break;
    case STATE_FACE_ON:
      if (millis() - lastFaceOnTime > PAUSE_DELAY_MILLIS) {
        sendCode(3); // Pause.
        state = STATE_TURNED_AWAY;
      }
      if (millis() - lastFaceSeenTime > TURN_OFF_DELAY_MILLIS) {
        sendCode(0); // Power off.
        state = STATE_NO_FACE;
      }
      break;
    case STATE_TURNED_AWAY:
      if (millis() - lastFaceSeenTime > TURN_OFF_DELAY_MILLIS) {
        sendCode(0); // Power off.
        state = STATE_NO_FACE;
      }
      if (is_face_on) {
        sendCode(2); // Play.
        state = STATE_FACE_ON;
      }
      break;
    default:
      Serial.println("Error - invalid state");
      break;
  }
  displayState(state, millis() - lastFaceSeenTime, millis() - lastFaceOnTime);
}
