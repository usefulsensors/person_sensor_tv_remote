#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
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

// === TCL Preset Codes ===
static const uint8_t kTCLProtocol = NEC;
static const uint8_t kTCLCodeBits = 32;

static const uint32_t kTCLCodes[] = {
  0x57E3E817, //Power
  0x57E304FB, //Mute
  0x57E332CD, //Play
  0x57E332CD, //Pause
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

#define STATE_NO_FACE 0
#define STATE_FACE_ON 1
#define STATE_TURNED_AWAY 2
#define PAUSE_DELAY_MILLIS 4000
#define TURN_OFF_DELAY_MILLIS 10000
#define POWER_RECT_OFFSET_Y 60
#define ATTENTION_RECT_OFFSET_Y 120
#define RECT_H 20

#define VERBOSE false

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

#define DEMO_MODE_ON_OFF 0
#define DEMO_MODE_MUTE_UNMUTE 1
#define DEMO_MODE_BBOXES 2
int demo_mode = DEMO_MODE_MUTE_UNMUTE;

void setup(){
  // Init Software I2C.
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setTimeout_ms(100);
  sw.begin();
  
  Serial.begin(BAUD_RATE);
  delay(250);
  if(VERBOSE) Serial.println("Booting person TV remote");
    CircuitPlayground.begin();
  initScreen();
  
  // Use Samsung codes by default.
  memcpy(&codeConfig.codeValues, kTCLCodes, sizeof(uint32_t) * kNumCodes);
  memset(&codeConfig.codeProtocols, kTCLProtocol, sizeof(uint8_t) * kNumCodes);
  memset(&codeConfig.codeBits, kTCLCodeBits, sizeof(uint8_t) * kNumCodes);
}

void loop(){
  static int previous_demo_mode = DEMO_MODE_MUTE_UNMUTE;
  // Switch between ON/OFF mode and MUTE/UNMUTE mode if in TV control modes.
  if (CircuitPlayground.leftButton()) {
    tft.fillRect(80, 190, 160, 20, ST77XX_BLACK);
    tft.setCursor(80, 190);
    if (demo_mode == DEMO_MODE_ON_OFF) {
      tft.print("MUTE/PAUSE");
      demo_mode = DEMO_MODE_MUTE_UNMUTE;
    } else if (demo_mode == DEMO_MODE_MUTE_UNMUTE) {
      tft.print("PAUSE/POWER");
      demo_mode = DEMO_MODE_ON_OFF;
    }
    delay(500);
  }
  // Switch between TV demo mode and 
  if(CircuitPlayground.rightButton()) {
    if (demo_mode == DEMO_MODE_ON_OFF || demo_mode == DEMO_MODE_MUTE_UNMUTE) {
      previous_demo_mode = demo_mode;
      demo_mode = DEMO_MODE_BBOXES;
    } else {
      demo_mode = previous_demo_mode;
    }
    drawBackground();
    delay(500);
  }
  person_sensor_results_t results;
  if(millis() - readTimestamp >= SAMPLE_PERIOD){
    person_sensor_results_t* results_ptr = &results;
    int8_t* results_bytes = (int8_t*) results_ptr;;
    int numBytes = sw.requestFrom(PERSON_SENSOR_I2C_ADDRESS, sizeof(person_sensor_results_t));
    for(int i = 0; i < sizeof(person_sensor_results_t); i++){
      results_bytes[i] = sw.read();
    }

    if (demo_mode == DEMO_MODE_BBOXES) {
      handleBbox(results);
    } else {
      handleSensorResults(results);
    }

    readTimestamp = millis();
  }
}

static int softwareScan(){
  int nDevices = 0;

  if(VERBOSE) Serial.println("Scanning...");

  int width_increment = 2;
  for(int address = 1; address < 127; address++){
    int rect_offset = address * 200 / 127;
    tft.fillRect(10 + rect_offset, POWER_RECT_OFFSET_Y, width_increment, RECT_H, ST77XX_GREEN);
    uint8_t startResult = sw.llStart((address << 1) + 1);
    sw.stop();

    if(startResult == 0){
      if(VERBOSE) Serial.print("I2C device found at address 0x");
      if(address < 16 && VERBOSE) Serial.print("0");
      if(VERBOSE) Serial.println(address,HEX);
      nDevices++;
    }
    delay(10);
  }

  if(nDevices == 0){
    if(VERBOSE) Serial.println("No I2C devices found");
  }else{
    if(VERBOSE) Serial.print(nDevices);
    if(VERBOSE) Serial.print(" devices found at time ");
    if(VERBOSE) Serial.println(millis());
  }

  return nDevices;
}

static void drawBackground() {
  tft.fillScreen(ST77XX_BLACK);
  if (demo_mode != DEMO_MODE_BBOXES) {
    // Draw bar background + text.
    tft.drawRect(8, POWER_RECT_OFFSET_Y-2, 204, RECT_H+4, ST77XX_GREEN);
    tft.drawRect(8, ATTENTION_RECT_OFFSET_Y-2, 204, RECT_H+4, ST77XX_GREEN);
    tft.setCursor(10, 0);
    tft.print("STATE:");
    tft.setCursor(10, 40);
    tft.print("PRESENCE:");
    tft.setCursor(10, 100);
    tft.print("ATTENTION:");
    tft.setCursor(10, 190);
    tft.print("MODE:");
    tft.setCursor(80, 190);
    if (demo_mode == DEMO_MODE_MUTE_UNMUTE) {
      tft.print("MUTE/PAUSE");
    } else {
      tft.print("PAUSE/POWER");
    }
  }
  tft.setCursor(10, 220);
  tft.print("NUM FACES:");
}

static void initScreen(){
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH); // Backlight on
  
  tft.init(240, 240);                // Init ST7789 240x240
  tft.setRotation(2);
  tft.setTextSize(2, 2);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextWrap(true);
  // Init message
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(8, POWER_RECT_OFFSET_Y-2, 204, RECT_H+4, ST77XX_GREEN);
  tft.setCursor(10, 0);
  tft.print("INITIALIZING...");
  int num_devices = softwareScan();
  tft.fillScreen(ST77XX_BLACK);
  if (num_devices < 1) {
    tft.setCursor(0, 0);
    tft.print("NO SENSOR DETECTED");
    while (true) {}
  }else{
    tft.setCursor(0, 0);
    tft.print("DEVICE FOUND AT 0x62");
    delay(1000);
  }
  drawBackground();
}

void sendCode(int code_idx) {
  CircuitPlayground.irSend.send(codeConfig.codeProtocols[code_idx], codeConfig.codeValues[code_idx], codeConfig.codeBits[code_idx]);
}

#define STATE_CHANGE_FACE 0
#define STATE_CHANGE_NO_FACE 1
#define STATE_CHANGE_FACE_ON 2
#define STATE_CHANGE_TURNED_AWAY 3
void handleStateChange(int state_change) {
  switch(state_change) {
    case STATE_CHANGE_FACE:
      if (demo_mode == DEMO_MODE_ON_OFF) {
        sendCode(0);
      } else {
        sendCode(2);
      }
      break;
    case STATE_CHANGE_NO_FACE:
      if (demo_mode == DEMO_MODE_ON_OFF) {
        sendCode(0);
      } else {
        sendCode(3);
      }
      break;
    case STATE_CHANGE_FACE_ON:
      if (demo_mode == DEMO_MODE_ON_OFF) {
        sendCode(2);
      } else {
        sendCode(1);
      }
      break;        
    case STATE_CHANGE_TURNED_AWAY:
      if (demo_mode == DEMO_MODE_ON_OFF) {
        sendCode(3);
      } else {
        sendCode(1);
      }
      break;
    default:
      break;
  }
}

void displayState(int state, int millis_since_face, int millis_since_face_on, bool state_changed, bool timer_reset, int num_faces) {
  static int previous_num_faces = -1;
  if (num_faces != previous_num_faces) {
    tft.fillRect(135, 220, 20, 20, ST77XX_BLACK);
    tft.setCursor(135, 220);
    tft.print(num_faces);
    previous_num_faces = num_faces;
  }
  if (state_changed || timer_reset) {
    if (state_changed) {
      tft.fillRect(90, 0, 80, 20, ST77XX_BLACK);
    }
    if (state_changed && state == STATE_NO_FACE) {
      tft.fillRect(10, POWER_RECT_OFFSET_Y, 200, RECT_H, ST77XX_BLACK);
    } else if (!(state_changed && state == STATE_TURNED_AWAY)) {
      tft.fillRect(10, POWER_RECT_OFFSET_Y, 200, RECT_H, ST77XX_GREEN);
    }
    if (state == STATE_TURNED_AWAY || state == STATE_NO_FACE) {
      tft.fillRect(10, ATTENTION_RECT_OFFSET_Y, 200, RECT_H, ST77XX_BLACK);
    } else {
      tft.fillRect(10, ATTENTION_RECT_OFFSET_Y, 200, RECT_H, ST77XX_GREEN);
    }
  }

  tft.setCursor(90, 0);
  if (state == STATE_NO_FACE) {
    if (demo_mode == DEMO_MODE_MUTE_UNMUTE) {
      tft.print("PAUSE");
    } else {
      tft.print("OFF");
    }
  } else if (state == STATE_FACE_ON) {
    tft.print("PLAY");
    int att_rect_w = millis_since_face_on * 200 / PAUSE_DELAY_MILLIS;
    tft.fillRect(210 - att_rect_w, ATTENTION_RECT_OFFSET_Y, att_rect_w, RECT_H, ST77XX_BLACK);
    int pwr_rect_w = millis_since_face * 200 / TURN_OFF_DELAY_MILLIS;
    tft.fillRect(210 - pwr_rect_w, POWER_RECT_OFFSET_Y, pwr_rect_w, RECT_H, ST77XX_BLACK);
  } else if (state == STATE_TURNED_AWAY) {
    if (demo_mode == DEMO_MODE_MUTE_UNMUTE) {
      tft.print("MUTE");
    } else {
      tft.print("PAUSE");
    }
    int pwr_rect_w = millis_since_face * 200 / TURN_OFF_DELAY_MILLIS;
    tft.fillRect(210 - pwr_rect_w, POWER_RECT_OFFSET_Y, pwr_rect_w, RECT_H, ST77XX_BLACK);
  }
}

void handleBbox(person_sensor_results_t results) {
  static int previous_num_faces = -1;
  static person_sensor_results_t previous_results = {};
  if (results.num_faces != previous_num_faces) {
    tft.fillRect(135, 220, 20, 20, ST77XX_BLACK);
    tft.setCursor(135, 220);
    tft.print(results.num_faces);
    previous_num_faces = results.num_faces;
  }

  // Erase previous results.
  for (int i=0; i<previous_results.num_faces; i++) {
    int x1 = previous_results.boxes[i].x1 * 240 / 256;
    int y1 = previous_results.boxes[i].y1 * 220 / 256;
    int x2 = previous_results.boxes[i].x2 * 240 / 256;
    int y2 = previous_results.boxes[i].y2 * 220 / 256;
    tft.drawRect(x1, y1, x2-x1, y2-y1, ST77XX_BLACK);
  }
  previous_results = results;
  // Draw new results.
  for (int i=0; i<results.num_faces; i++) {
    int x1 = results.boxes[i].x1 * 240 / 256;
    int y1 = results.boxes[i].y1 * 220 / 256;
    int x2 = results.boxes[i].x2 * 240 / 256;
    int y2 = results.boxes[i].y2 * 220 / 256;
    if (results.boxes[i].face_on) {
      tft.drawRect(x1, y1, x2-x1, y2-y1, ST77XX_GREEN);
    } else {
      tft.drawRect(x1, y1, x2-x1, y2-y1, ST77XX_BLUE);
    }
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

  bool state_change = false;
  bool timer_reset = state == STATE_FACE_ON && is_face_on || state == STATE_TURNED_AWAY && has_face;
  lastFaceSeenTime = has_face ? millis() : lastFaceSeenTime;
  lastFaceOnTime = is_face_on ? millis() : lastFaceOnTime;
  switch(state) {
    case STATE_NO_FACE:
      if (has_face) {
        state_change = true;
        handleStateChange(STATE_CHANGE_FACE);
        lastFaceSeenTime = millis();
        if (is_face_on) {
          lastFaceOnTime = millis();
          state = STATE_FACE_ON;
          delay(100);
          handleStateChange(STATE_CHANGE_FACE_ON);
        } else {
          state = STATE_TURNED_AWAY;
        }
      }
      break;
    case STATE_FACE_ON:
      if (millis() - lastFaceOnTime > PAUSE_DELAY_MILLIS) {
        state_change = true;
        handleStateChange(STATE_CHANGE_TURNED_AWAY);
        state = STATE_TURNED_AWAY;
      }
      if (millis() - lastFaceSeenTime > TURN_OFF_DELAY_MILLIS) {
        state_change = true;
        handleStateChange(STATE_CHANGE_NO_FACE);

        state = STATE_NO_FACE;
      }
      break;
    case STATE_TURNED_AWAY:
      if (millis() - lastFaceSeenTime > TURN_OFF_DELAY_MILLIS) {
        state_change = true;
        handleStateChange(STATE_CHANGE_NO_FACE);

        state = STATE_NO_FACE;
      }
      if (is_face_on) {
        state_change = true;
        handleStateChange(STATE_CHANGE_FACE_ON);
        state = STATE_FACE_ON;
      }
      break;
    default:
      if(VERBOSE) Serial.println("Error - invalid state");
      break;
  }
  displayState(state, millis() - lastFaceSeenTime, millis() - lastFaceOnTime, state_change, timer_reset, results.num_faces);
}
