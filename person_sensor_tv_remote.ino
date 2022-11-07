/* Infrared_Send.ino Example sketch for IRLib2 and Circuit Playground Express
   Illustrates how to transmit an IR signal whenever you do push one of the
   built-in pushbuttons.
*/
#include <Adafruit_CircuitPlayground.h>

#include "person_sensor.h"

#if !defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS)
  #error "Infrared support is only for the Circuit Playground Express, it doesn't work with the Classic version"
#endif

//Defines for a Samsung TV using NECx protocol
#define MY_PROTOCOL NECX
#define MY_BITS 32
#define MY_MUTE 0xE0E0F00F
#define MY_POWER 0xE0E040BF
#define MY_PLAY 0xE0E0E21D
#define MY_PAUSE 0xE0E052AD

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

// We play some tricks to update the recorded values in flash so they persist.
// See https://learn.adafruit.com/adafruit-feather-m0-basic-proto/adapting-sketches-to-m0#storing-data-in-flash-2677106.
const uint8_t playCodeProtocolFlash = NECX;
uint8_t* playCodeProtocol = (uint8_t*)(&playCodeProtocolFlash);

const uint32_t playCodeValueFlash = 0xE0E0E21D;
uint32_t* playCodeValue = (uint32_t*)(&playCodeValueFlash);

const uint8_t playCodeBitsFlash = 32;
uint8_t* playCodeBits = (uint8_t*)(&playCodeBitsFlash);

const uint8_t pauseCodeProtocolFlash = NECX;
uint8_t* pauseCodeProtocol = (uint8_t*)(&playCodeProtocolFlash);

const uint32_t pauseCodeValueFlash = 0xE0E052AD;
uint32_t* pauseCodeValue = (uint32_t*)(&pauseCodeValueFlash);

const uint8_t pauseCodeBitsFlash = 32;
uint8_t* pauseCodeBits = (uint8_t*)(&pauseCodeBitsFlash);

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

void setup() {
  CircuitPlayground.begin();
  ir_setup();
  Wire.begin();
}

void ir_loop() {
  if (CircuitPlayground.irReceiver.getResults()) {
    CircuitPlayground.irDecoder.decode();
    receiveCode();
    CircuitPlayground.irReceiver.enableIRIn(); // Re-enable receiver
  }
  if (gotNew) {
    if (waitingForPlay) {
      *playCodeProtocol = codeProtocol;
      *playCodeValue = codeValue;
      *playCodeBits = codeBits;
      waitingForPlay = false;
      waitingForPause = true;
      Serial.print(F("Play = 0x"));
      Serial.println(*playCodeValue, HEX);
    } else if (waitingForPause) {
      *pauseCodeProtocol = codeProtocol;
      *pauseCodeValue = codeValue;
      *pauseCodeBits = codeBits;
      waitingForPause = false;
      Serial.print(F("Pause = 0x"));
      Serial.println(*pauseCodeValue, HEX);
    }
    gotNew = false;
  }
}

void person_sensor_loop() {
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
      CircuitPlayground.irSend.send(*pauseCodeProtocol, *pauseCodeValue, *pauseCodeBits);
      isPlaying = false;
    }
  } else if (hasFace && (timeFaceSeen > playDelayCount)) {
    CircuitPlayground.irSend.send(*playCodeProtocol, *playCodeValue, *playCodeBits);
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
    while (CircuitPlayground.leftButton()) {}//wait until button released
  }
  // If the right button is pressed send a play/pause code.
  if (CircuitPlayground.rightButton()) {
    if (isPaused) {
      CircuitPlayground.irSend.send(*playCodeProtocol, *playCodeValue, *playCodeBits);
      Serial.print(F("Sending play = 0x"));
      Serial.println(*playCodeValue, HEX);      
      isPaused = false;
    } else {
      CircuitPlayground.irSend.send(*pauseCodeProtocol, *pauseCodeValue, *pauseCodeBits);      
      Serial.print(F("Sending pause = 0x"));
      Serial.println(*pauseCodeValue, HEX);      
      isPaused = true;
    }
    while (CircuitPlayground.rightButton()) {}//wait until button released
  }
}
