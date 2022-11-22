/* Infrared_Send.ino Example sketch for IRLib2 and Circuit Playground Express
   Illustrates how to transmit an IR signal whenever you do push one of the
   built-in pushbuttons.
*/
#include <math.h>

#include <Adafruit_CircuitPlayground.h>

#include <FlashStorage.h>

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
const int32_t SAMPLE_DELAY_MS = 200;

//These flags keep track of whether we received the first code 
//and if we have have received a new different code from a previous one.
bool gotOne = false;
bool gotNew = false; 

uint8_t codeProtocol = UNKNOWN;
uint32_t codeValue = 0;
uint8_t codeBits = 0;

typedef struct __attribute__ ((__packed__)) {
  bool isValid;
  uint8_t playCodeProtocol;
  uint32_t playCodeValue;
  uint8_t playCodeBits;

  uint8_t pauseCodeProtocol;
  uint32_t pauseCodeValue;
  uint8_t pauseCodeBits;
} CodeConfig_t;

const CodeConfig_t samsungCodes = {
  .playCodeProtocol = NECX,
  .playCodeValue = 0xE0E0E21D,
  .playCodeBits = 32,
  .pauseCodeProtocol = NECX,
  .pauseCodeValue = 0xE0E052AD,
  .pauseCodeBits = 32,
};

CodeConfig_t codeConfig;

FlashStorage(flashStore, CodeConfig_t);

bool waitingForPlay = false;
bool waitingForPause = false;

bool isPaused = false;

// How long to wait to pause and play. Alter these to adjust the behavior.
const int32_t pauseDelaySeconds = 5;
const int32_t playDelaySeconds = 1;

// Convert timeout seconds into loop iteration counts.
const int32_t pauseDelayCount = (pauseDelaySeconds * 1000) / SAMPLE_DELAY_MS;
const int32_t playDelayCount = (playDelaySeconds * 1000) / SAMPLE_DELAY_MS;

int32_t timeSinceFaceSeen = 0;
int32_t timeFaceSeen = 0;
bool isPlaying = true;

const int audioSampleRate = 22050;

void ir_setup() {
  gotOne=false;
  gotNew=false;
  Serial.begin(9600);
  delay(2000);
  CircuitPlayground.irReceiver.enableIRIn(); // Start the receiver
}

// Stores the code for later playback
void receiveCode(void) {
  codeProtocol = CircuitPlayground.irDecoder.protocolNum;
  if (codeProtocol == UNKNOWN) {
    return;
  }
  
  Serial.print(F("Received "));
  Serial.print(Pnames(codeProtocol));
  if (CircuitPlayground.irDecoder.value == REPEAT_CODE) {
    // Don't record a NEC repeat value as that's useless.
    Serial.println(F("repeat; ignoring."));
    return;
  }

  codeValue = CircuitPlayground.irDecoder.value;
  codeBits = CircuitPlayground.irDecoder.bits;
  gotNew=true;
  gotOne=true;
  Serial.print(F(" Value:0x"));
  Serial.println(codeValue, HEX);
}

void flash_read_config() {
  codeConfig = flashStore.read();
  if (codeConfig.isValid) {
    Serial.println("Read config from flash");
  } else {
    Serial.println("No config found, writing to flash");
    codeConfig = samsungCodes;
    codeConfig.isValid = true;
  }
  Serial.print(F("Read play = 0x"));
  Serial.println(codeConfig.playCodeValue, HEX);
  Serial.print(F("Read pause = 0x"));
  Serial.println(codeConfig.pauseCodeValue, HEX);
}

void flash_write_config() {
  flashStore.write(codeConfig);
}

// Macro to automatically look up the right variable name based on a phrase, and play it.
#define PLAY_AUDIO(NAME) do { play_audio(g_##NAME##_data, g_##NAME##_data_len); } while (false)

void play_audio(const uint8_t* bytes, int bytes_count) {
  CircuitPlayground.speaker.playSound(bytes, bytes_count, audioSampleRate, false);
}

void setup() {
  CircuitPlayground.begin();
  CircuitPlayground.clearPixels();    
  ir_setup();
  flash_read_config();
  Wire.begin();
}

bool is_recording() {
  return (waitingForPlay || waitingForPause);
}

void ir_loop() {
  static int pixelIndex = 0;
  static int pixelInc = 1;
  if (is_recording()) {
    int8_t red;
    int8_t green;
    int8_t blue;
    if (waitingForPlay) {
      red = 255;
      green = 255;
      blue = 0;
    } else {      
      red = 0;
      green = 0;
      blue = 255;
    }
    CircuitPlayground.clearPixels();
    CircuitPlayground.setPixelColor(pixelIndex, red, green, blue);
    pixelIndex = (pixelIndex + pixelInc + 10) % 10;
    delay(SAMPLE_DELAY_MS);
  }
  
  if (CircuitPlayground.irReceiver.getResults()) {
    CircuitPlayground.irDecoder.decode();
    receiveCode();
    CircuitPlayground.irReceiver.enableIRIn(); // Re-enable receiver
  }
  if (gotNew) {
    if (waitingForPlay) {
      codeConfig.playCodeProtocol = codeProtocol;
      codeConfig.playCodeValue = codeValue;
      codeConfig.playCodeBits = codeBits;
      waitingForPlay = false;
      waitingForPause = true;
      PLAY_AUDIO(press);
      PLAY_AUDIO(pause);
      pixelInc = -pixelInc;
      Serial.print(F("Play = 0x"));
      Serial.println(codeConfig.playCodeValue, HEX);
    } else if (waitingForPause) {
      codeConfig.pauseCodeProtocol = codeProtocol;
      codeConfig.pauseCodeValue = codeValue;
      codeConfig.pauseCodeBits = codeBits;
      waitingForPause = false;
      PLAY_AUDIO(done);
      pixelInc = -pixelInc;
      CircuitPlayground.clearPixels();    
      Serial.print(F("Pause = 0x"));
      Serial.println(codeConfig.pauseCodeValue, HEX);
      flash_write_config();
    }
    gotNew = false;
  }
}

void person_sensor_loop() {
  if (is_recording()) {
    return;
  }
  person_sensor_results_t results = {};
  // Perform a read action on the I2C address of the sensor to get the
  // current face information detected.
  if (!person_sensor_read(&results)) {
    Serial.println("No person sensor results found on the i2c bus");
    delay(SAMPLE_DELAY_MS);
    return;
  }

  const bool hasFace = (results.num_faces > 0);
  if (hasFace) {
    timeSinceFaceSeen = 0;
    timeFaceSeen += 1;
  } else {
    timeSinceFaceSeen += 1;
    timeFaceSeen = 0;
  }
  if (isPlaying) {
    if (timeSinceFaceSeen == pauseDelayCount) {
      CircuitPlayground.irSend.send(codeConfig.pauseCodeProtocol, codeConfig.pauseCodeValue, codeConfig.pauseCodeBits);
      PLAY_AUDIO(pause);
      isPlaying = false;
    }
  } else if (hasFace && (timeFaceSeen > playDelayCount)) {
    CircuitPlayground.irSend.send(codeConfig.playCodeProtocol, codeConfig.playCodeValue, codeConfig.playCodeBits);
    PLAY_AUDIO(play);
    isPlaying = true;
  }

  delay(SAMPLE_DELAY_MS);
}

void loop() {  
  ir_loop();
  person_sensor_loop();
  
  // If the left button is pressed send a mute code.
  if (CircuitPlayground.leftButton()) {
    waitingForPlay = true;
    PLAY_AUDIO(recording);
    while (CircuitPlayground.leftButton()) {}//wait until button released
    PLAY_AUDIO(press);
    PLAY_AUDIO(play);
  }
  // If the right button is pressed send a play/pause code.
  if (CircuitPlayground.rightButton()) {
    if (isPaused) {
      CircuitPlayground.irSend.send(codeConfig.playCodeProtocol, codeConfig.playCodeValue, codeConfig.playCodeBits);
      Serial.print(F("Sending play = 0x"));
      Serial.println(codeConfig.playCodeValue, HEX);      
      isPaused = false;
    } else {
      CircuitPlayground.irSend.send(codeConfig.pauseCodeProtocol, codeConfig.pauseCodeValue, codeConfig.pauseCodeBits);      
      Serial.print(F("Sending pause = 0x"));
      Serial.println(codeConfig.pauseCodeValue, HEX);      
      isPaused = true;
    }
    while (CircuitPlayground.rightButton()) {}//wait until button released
  }
}
