/* Infrared_Send.ino Example sketch for IRLib2 and Circuit Playground Express
   Illustrates how to transmit an IR signal whenever you do push one of the
   built-in pushbuttons.
*/
#include <math.h>

#include <Adafruit_CircuitPlayground.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

#include "person_sensor.h"

#include "audio_data_done.h"
#include "audio_data_down.h"
#include "audio_data_enter.h"
#include "audio_data_left.h"
#include "audio_data_lg.h"
#include "audio_data_mute.h"
#include "audio_data_pause.h"
#include "audio_data_play.h"
#include "audio_data_press.h"
#include "audio_data_recording.h"
#include "audio_data_right.h"
#include "audio_data_samsung.h"
#include "audio_data_sony.h"
#include "audio_data_tcl.h"
#include "audio_data_up.h"
#include "audio_data_vizio.h"

#if !defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS)
  #error "Infrared support is only for the Circuit Playground Express, it doesn't work with the Classic version"
#endif

// How long to wait between reading the sensor. The sensor can be read as
// frequently as you like, but the results only change at about 5FPS, so
// waiting for 200ms is reasonable.
const int32_t SAMPLE_DELAY_MS = 250;

//These flags keep track of whether we received the first code 
//and if we have have received a new different code from a previous one.
bool gotNew = false; 

uint8_t codeProtocol = UNKNOWN;
uint32_t codeValue = 0;
uint8_t codeBits = 0;
static int codeIndex = 0;

static const int kNumCodes = 6;

typedef struct __attribute__ ((__packed__)) {
  uint8_t codeProtocol;
  uint32_t powerCode;
  uint32_t playCode;
  uint32_t muteCode;
  uint32_t downCode;
  uint32_t upCode;
  uint32_t backCode;
  uint8_t codeBits;
} CodeConfig_t;

#define MY_MUTE 0xE0E0F00F
#define MY_POWER 0xE0E040BF
#define MY_PLAY 0xE0E0E21D
#define MY_PAUSE 0xE0E052AD

// === Samsung Preset Codes ===
static const CodeConfig_t kSamsungCodes = {
  .codeProtocol = NECX,
  .powerCode = 0xE0E040BF,
  .playCode = 0xE0E016E9,
  .muteCode = 0xE0E0F00F,
  .codeBits = 32
};

// === TCL Preset Codes ===
static const CodeConfig_t kTCLCodes = {
  .codeProtocol = NEC,
  .powerCode = 0x57E3E916,
  .playCode = 0x57E354AB,
  .muteCode = 0x57E304FB,
  .downCode = 0x57E3CD32,
  .upCode = 0x57E39966,
  .backCode = 0x57E36798,
  .codeBits = 32
};

CodeConfig_t codeConfig;

// === LG Preset Codes ===

// === Sony Preset Codes ===

int delays[kNumCodes] = {};
static const int gesture_delays[kNumCodes] = {
  20, // on/off
  4,  // up
  4,  // down
  12, // enter
  12, // back
  20, // mute
};

bool commandProgrammingMode = false;

Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;
File32 myFile;

#define FLASH_BLOCK_SIZE (256)
const char* config_file_name = "config.bin";

const int audioSampleRate = 22050;

#define PLAY_AUDIO(NAME) do { play_audio(g_##NAME##_data, g_##NAME##_data_len); } while (false)

void play_audio(const uint8_t* bytes, int bytes_count) {
  CircuitPlayground.speaker.playSound(bytes, bytes_count, audioSampleRate, false);
}

void ir_setup() {
  gotNew=false;
  Serial.begin(9600);
  delay(2000);
  CircuitPlayground.irReceiver.enableIRIn(); // Start the receiver
}

// Stores the code for later playback
void receiveCode(void) {
  static uint32_t last_code = 0;
  codeProtocol = CircuitPlayground.irDecoder.protocolNum;
  if (codeProtocol == UNKNOWN) {
    return;
  }

  if (CircuitPlayground.irDecoder.value == REPEAT_CODE) {
    // Don't record a NEC repeat value as that's useless.
    Serial.println(F("repeat; ignoring."));
    return;
  }

  if (CircuitPlayground.irDecoder.value != last_code) {
    gotNew=true;
    codeValue = CircuitPlayground.irDecoder.value;
    codeBits = CircuitPlayground.irDecoder.bits;
    last_code = codeValue;
    Serial.print(F("Received "));
    Serial.print(Pnames(codeProtocol));
    Serial.print(F(" Value:0x"));
    Serial.println(codeValue, HEX);
  }
}

void useSamsungCodes() {
  memcpy(&codeConfig, &kSamsungCodes, sizeof(CodeConfig_t));
  PLAY_AUDIO(samsung);
  flash_write_config();
}

void useTCLCodes() {
  memcpy(&codeConfig, &kTCLCodes, sizeof(CodeConfig_t));
  PLAY_AUDIO(tcl);
  flash_write_config();
}

void flash_read_config() {
  myFile = fatfs.open(config_file_name);
  if (myFile) {
    Serial.print("Reading config from ");
    Serial.println(config_file_name);
    Serial.print("Length is ");
    Serial.println(myFile.fileSize());
    myFile.read((uint*)(&codeConfig), sizeof(codeConfig)); 
    myFile.close();
  } else {
    Serial.print("No config found, Defaulting to Samsung Codes");
    //commandProgrammingMode = true;
    //useSamsungCodes();
    useTCLCodes();
  }
}

void flash_write_config() {
    myFile = fatfs.open(config_file_name, (O_RDWR | O_CREAT | O_TRUNC));
    if (!myFile) {
      Serial.print("Failed to open for writing: ");
      Serial.println(config_file_name);
      return;
    }
    myFile.write((uint*)(&codeConfig), sizeof(codeConfig)); 
    myFile.close();
    flash_read_config();
}

void flash_setup() {
  flash.begin();

  // Open file system on the flash
  if ( !fatfs.begin(&flash) ) {
    Serial.println("Error: filesystem does not exist. Please try SdFat_format example to make one.");
    while(1)
    {
      yield();
      delay(1);
    }
  }

  flash_read_config();
}

void setup() {
  CircuitPlayground.begin();
  CircuitPlayground.clearPixels();    
  ir_setup();
  flash_setup();
  Wire.begin();
}

void ir_loop() {
  if (!commandProgrammingMode) {
    CircuitPlayground.setPixelColor(0, 0, 0, 255);  
  }
  if (CircuitPlayground.irReceiver.getResults()) {
    CircuitPlayground.irDecoder.decode();
    receiveCode();
    CircuitPlayground.irReceiver.enableIRIn(); // Re-enable receiver
  }
  if (gotNew) {
    if (commandProgrammingMode) {
      codeConfig.codeProtocol = codeProtocol;
      switch(codeIndex) {
        case 0:
          codeConfig.powerCode = codeValue;
          break;
        case 1: 
          codeConfig.upCode = codeValue;
          break;
        case 2: 
          codeConfig.downCode = codeValue;
          break;
        case 3: 
          codeConfig.playCode = codeValue;
          break;
        case 4: 
          codeConfig.backCode = codeValue;
          break;
        case 5:
          codeConfig.muteCode = codeValue;
          break;
      }
      codeConfig.codeBits = codeBits;
      CircuitPlayground.setPixelColor(codeIndex, 255, 255, 0);

      Serial.print(F("New Code = 0x"));
      Serial.println(codeValue, HEX);
      Serial.print("num codex received:"); Serial.println(codeIndex);
      if (++codeIndex == kNumCodes) {
        CircuitPlayground.clearPixels();
        commandProgrammingMode = false;
        Serial.println("got all commands, leaving programming mode.");
        Serial.flush();
        flash_write_config();
      } else {
        delay(200);
      }
    }
    gotNew = false;
  }
}

int gestureIdToCodeIndex(int gesture_idx) {
  if (gesture_idx == 13) {
    return 0; // on/off
  } else if (gesture_idx == 7) {
    return 3; // ok
  } else if (gesture_idx == 6) {
    return 5; // mute
  } else if (gesture_idx == 5) {
    return 1; // up
  } else if (gesture_idx == 2) {
    return 2; // down
  } else if (gesture_idx == 3) {
    return 4; // back
  }
  return -1;
}

void gesture_audio(int gesture_idx) {
  switch(gesture_idx) {
    case 0: // Power
      PLAY_AUDIO(press);
      break;
    case 1: // Up
      PLAY_AUDIO(up);
      break;
    case 2: // Down
      PLAY_AUDIO(down);
      break;
    case 3: // Enter
      PLAY_AUDIO(play);
      break;
    case 4: // Back
      PLAY_AUDIO(pause);
      break;
    case 5: // Mute
      PLAY_AUDIO(mute);
      break;
  }
}

void person_sensor_loop() {
  // disable transmit when in programming mode to avoid cross-talk between rx and tx IR LEDs.
  if (commandProgrammingMode) {
    return;
  }
  person_sensor_results_t results = {};
  // Perform a read action on the I2C address of the sensor to get the
  // current hand information detected.
  if (!person_sensor_read(&results)) {
    Serial.println("No person sensor results found on the i2c bus");
    delay(SAMPLE_DELAY_MS);
    return;
  }

  const bool hasHand = (results.num_hands > 0);
  if (hasHand) {
    int code_idx = gestureIdToCodeIndex(results.hands[0].gesture_class);
    if (code_idx >= 0 && delays[code_idx] == 0) {
      Serial.print("sending code "); Serial.println(code_idx);
      switch(code_idx) {
        case 0: // Power
          CircuitPlayground.irSend.send(codeConfig.codeProtocol, codeConfig.powerCode, codeConfig.codeBits);
          break;
        case 1: // Up
          CircuitPlayground.irSend.send(codeConfig.codeProtocol, codeConfig.upCode, codeConfig.codeBits);
          break;
        case 2: // Down
          CircuitPlayground.irSend.send(codeConfig.codeProtocol, codeConfig.downCode, codeConfig.codeBits);
          break;
        case 3: // Enter
          CircuitPlayground.irSend.send(codeConfig.codeProtocol, codeConfig.playCode, codeConfig.codeBits);
          break;
        case 4: // Back
          CircuitPlayground.irSend.send(codeConfig.codeProtocol, codeConfig.backCode, codeConfig.codeBits);
          break;
        case 5: // Mute
          CircuitPlayground.irSend.send(codeConfig.codeProtocol, codeConfig.muteCode, codeConfig.codeBits);
          break;
      }
      delays[code_idx] = gesture_delays[code_idx];
      gesture_audio(code_idx);
    }
  }

  for (int i=0; i<kNumCodes; i++) {
    delays[i] -= 1;
    if (delays[i] < 0) {
      delays[i] = 0;
    }
  }
  delay(SAMPLE_DELAY_MS);
}

void loop() {
  static const int kResetDelay = 2000;
  ir_loop();
  person_sensor_loop();
  
  // If the left button is pressed, enter programming mode.
  if (CircuitPlayground.leftButton()) {
    int start_time = millis();
    while (CircuitPlayground.rightButton()) {}
    if (millis() - start_time > kResetDelay) {
      for (int i=0; i<10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 255, 0);
      }
      useTCLCodes();
      delay(200);
      for (int i=0; i<10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
      }
      return;
    }
    Serial.println("entering programming mode.");
    commandProgrammingMode = true;
    codeIndex = 0;
    CircuitPlayground.clearPixels();
    gotNew = false;
    while (CircuitPlayground.leftButton()) {}//wait until button released
  }
}
