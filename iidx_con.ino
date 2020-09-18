/* Arduino IIDX Doubles Controller Code for Leonardo
 *  
 *  Based on code written by Knuckles Lee
 *  https://github.com/knuckleslee/RhythmCodes/tree/master/1E11B11LED_iidx
 *  
 * 2 Encoders + 14 Buttons + 14 HID controlable LED + WS28121B strip for turntables
 * 
 * Arduino Joystick Library
 * https://github.com/MHeironimus/ArduinoJoystickLibrary/
 * 
 * mon's Arduino-HID-Lighting
 * https://github.com/mon/Arduino-HID-Lighting
 * 
 * Adafruit PWM Servo Driver Library
 * https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * 
 * Fast LED Library
 * https://github.com/FastLED/FastLED
 * 
 * for use with Arduino Leonardo and one Adafruit 16-Channel 12-Bit PWM/Servo Driver
 */
 
// I2C pins needed for the LEDs

#include <Joystick.h>

#include "PluggableUSB.h"
#include "HID.h"

#include <FastLED.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

// LED Strip Setup
#define LED_PIN     14
#define NUM_LEDS    40
CRGB leds[NUM_LEDS];

// HID lighting setup
typedef struct {
  uint8_t brightness;
} SingleLED;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} RGBLed;

// The single LEDs will be first in BTools
// The RGB LEDs will come afterwards, with R/G/B individually
#define NUMBER_OF_SINGLE 14
#define NUMBER_OF_RGB 1

// Number of colors you can change your turntable to
#define NUMBER_OF_TT_COLORS 9
int TTCount = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Joystick setup
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD, 14, 0,
 false, false, false, true, true, false, false, false, false, false, false);

// Assign our IO parameters
const int PULSE = 70;  //number of pulses per revolution of encoders (600) or gear teeth (20)
byte EncPins[]    = { 1, 5, 0, 4};
byte ButtonPins[] = { 22, 23, 19, 20, 18, 13, 21, 11, 8, 6, 7, 9, 10, 12 };
int lightVals[14] = { 0 };
boolean updateLights = false;
const byte ButtonCount = sizeof(ButtonPins) / sizeof(ButtonPins[0]);
const byte EncPinCount = sizeof(EncPins) / sizeof(EncPins[0]);
const byte EncCount = EncPinCount / 2;
int enc[EncCount]={0};
boolean hidMode, state[EncCount]={false}, set[EncPinCount]={false};
int ReportDelay = 700;
unsigned long ReportRate;

void SetLEDColor();

void setup() {
  // load turntable color from memory
  EEPROM.get(0, TTCount);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  
  Joystick.begin(false);
  Joystick.setRxAxisRange(-PULSE/2, PULSE/2-1);
  Joystick.setRyAxisRange(-PULSE/2, PULSE/2-1);
  
  // setup I/O for pins
  for(int i=0;i<ButtonCount;i++) {
    pinMode(ButtonPins[i], INPUT_PULLUP);
  }

  for(int i=0;i<EncPinCount;i++) {
    pinMode(EncPins[i], INPUT_PULLUP);
  }

  //light mode detection
  hidMode = digitalRead(ButtonPins[0]);

  if(hidMode == false){
    SetLEDColor();
  }

  //setup interrupts
  attachInterrupt(digitalPinToInterrupt(EncPins[0]), doEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncPins[2]), doEncoder1, CHANGE);

  // setup pwm driver
  pwm.begin();
  pwm.setPWMFreq(1000);
}

void loop() {
  ReportRate = micros() ;
  
  // read buttons
  for(int i = 0; i < ButtonCount;i++) {
    Joystick.setButton (i, !(digitalRead(ButtonPins[i])));
  }

  //change turntable color
  if(digitalRead(ButtonPins[0]) == 0 && digitalRead(ButtonPins[1]) == 0 && digitalRead(ButtonPins[2]) == 0 && digitalRead(ButtonPins[3]) == 0 && digitalRead(ButtonPins[4]) == 0 && digitalRead(ButtonPins[5]) == 0 && digitalRead(ButtonPins[6]) == 0
     && digitalRead(ButtonPins[7]) == 0 && digitalRead(ButtonPins[8]) == 0 && digitalRead(ButtonPins[9]) == 0 && digitalRead(ButtonPins[10]) == 0 && digitalRead(ButtonPins[11]) == 0 && digitalRead(ButtonPins[12]) == 0 && digitalRead(ButtonPins[13]) == 0 
     && hidMode == false){
    TTCount++;
    delay(500);
    if(TTCount == NUMBER_OF_TT_COLORS){
      TTCount = 0;
    }
    SetLEDColor();
    Serial.println(TTCount) ;
  }

  //read encoders, detect overflow and rollover
  for(int i=0; i<EncCount; i++) {
    if(enc[i] < -PULSE/2 || enc[i] > PULSE/2-1)
      enc[i] = constrain (enc[i]*-1, -PULSE/2, PULSE/2-1);
  }
  
  Joystick.setRxAxis(enc[0]);
  Joystick.setRyAxis(enc[1]);

  //report
  Joystick.sendState();
  delayMicroseconds(ReportDelay);
  
  // reactive lighting if not in HID mode
  if(hidMode == false){
    for(int i = 0; i < ButtonCount; i++) {
      if(digitalRead(ButtonPins[i]) == 0){
        pwm.setPWM(i, 4096, 0);
      }else{
        pwm.setPWM(i, 0, 4096);
      }
    }
  }
  
  // hid Lighting
  if (updateLights && hidMode == true){
    for (int i = 0; i < NUMBER_OF_SINGLE; i++) {
      pwm.setPWM(i, 0, lightVals[i]);
    }
    FastLED.show();
    updateLights = false;
  }
}

//Interrupts
void doEncoder0() {
  if(state[0] == false && digitalRead(EncPins[0]) == LOW) {
    set[0] = digitalRead(EncPins[1]);
    state[0] = true;
  }
  if(state[0] == true && digitalRead(EncPins[0]) == HIGH) {
    set[1] = !digitalRead(EncPins[1]);
    if(set[0] == true && set[1] == true) {
      enc[0]++;
    }
    if(set[0] == false && set[1] == false) {
      enc[0]--;
    }
    state[0] = false;
  }
}

void doEncoder1() {
  if(state[1] == false && digitalRead(EncPins[2]) == LOW) {
    set[2] = digitalRead(EncPins[3]);
    state[1] = true;
  }
  if(state[1] == true && digitalRead(EncPins[2]) == HIGH) {
    set[3] = !digitalRead(EncPins[3]);
    if(set[2] == true && set[3] == true) {
      enc[1]++;
    }
    if(set[2] == false && set[3] == false) {
      enc[1]--;
    }
    state[1] = false;
  }
}

void SetLEDColor(){
  //Turntable LEDs
  if(TTCount == 0){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::White;
    }
  }
  if(TTCount == 1){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::Blue;
    }
  }
  if(TTCount == 2){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::Purple;
    }
  }
  if(TTCount == 3){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB(255, 20, 255);
    }
  }
  if(TTCount == 4){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::Red;
    }
  }
  if(TTCount == 5){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::Orange;
    }
  }
  if(TTCount == 6){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::Yellow;
    }
  }
  if(TTCount == 7){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::Green;
    }
  }
  if(TTCount == 8){
    for(int i = 0; i < NUM_LEDS; ++i){
      leds[i] = CRGB::Black;
    }
  }
  FastLED.show();
  EEPROM.put(0, TTCount);
}

void light_update(SingleLED* single_leds, RGBLed* rgb_leds) {
  for(int i = 0; i < NUMBER_OF_SINGLE; i++){
    lightVals[i] = (int) map(single_leds[i].brightness, 0, 255, 0, 4095);
  }
  for(int i = 0; i < NUMBER_OF_RGB; i++) {
    leds[i] = CRGB(rgb_leds[i].r, rgb_leds[i].g, rgb_leds[i].b);
  }
  for(int i = 1; i < NUM_LEDS; i++){
    leds[i] = leds[0];
  }
  updateLights = true;
}

// ******************************
// don't need to edit below here

#define NUMBER_OF_LIGHTS (NUMBER_OF_SINGLE + NUMBER_OF_RGB*3)
#if NUMBER_OF_LIGHTS > 63
  #error You must have less than 64 lights
#endif

union {
  struct {
    SingleLED singles[NUMBER_OF_SINGLE];
    RGBLed rgb[NUMBER_OF_RGB];
  } leds;
  uint8_t raw[NUMBER_OF_LIGHTS];
} led_data;

static const uint8_t PROGMEM _hidReportLEDs[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x00,                    // USAGE (Undefined)
    0xa1, 0x01,                    // COLLECTION (Application)
    // Globals
    0x95, NUMBER_OF_LIGHTS,        //   REPORT_COUNT
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x05, 0x0a,                    //   USAGE_PAGE (Ordinals)
    // Locals
    0x19, 0x01,                    //   USAGE_MINIMUM (Instance 1)
    0x29, NUMBER_OF_LIGHTS,        //   USAGE_MAXIMUM (Instance n)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    // BTools needs at least 1 input to work properly
    0x19, 0x01,                    //   USAGE_MINIMUM (Instance 1)
    0x29, 0x01,                    //   USAGE_MAXIMUM (Instance 1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0xc0                           // END_COLLECTION
};

// This is almost entirely copied from NicoHood's wonderful RawHID example
// Trimmed to the bare minimum
// https://github.com/NicoHood/HID/blob/master/src/SingleReport/RawHID.cpp
class HIDLED_ : public PluggableUSBModule {

  uint8_t epType[1];
  
  public:
    HIDLED_(void) : PluggableUSBModule(1, 1, epType) {
      epType[0] = EP_TYPE_INTERRUPT_IN;
      PluggableUSB().plug(this);
    }

    int getInterface(uint8_t* interfaceCount) {
      *interfaceCount += 1; // uses 1
      HIDDescriptor hidInterface = {
        D_INTERFACE(pluggedInterface, 1, USB_DEVICE_CLASS_HUMAN_INTERFACE, HID_SUBCLASS_NONE, HID_PROTOCOL_NONE),
        D_HIDREPORT(sizeof(_hidReportLEDs)),
        D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint), USB_ENDPOINT_TYPE_INTERRUPT, USB_EP_SIZE, 16)
      };
      return USB_SendControl(0, &hidInterface, sizeof(hidInterface));
    }
    
    int getDescriptor(USBSetup& setup)
    {
      // Check if this is a HID Class Descriptor request
      if (setup.bmRequestType != REQUEST_DEVICETOHOST_STANDARD_INTERFACE) { return 0; }
      if (setup.wValueH != HID_REPORT_DESCRIPTOR_TYPE) { return 0; }
    
      // In a HID Class Descriptor wIndex contains the interface number
      if (setup.wIndex != pluggedInterface) { return 0; }
    
      return USB_SendControl(TRANSFER_PGM, _hidReportLEDs, sizeof(_hidReportLEDs));
    }
    
    bool setup(USBSetup& setup)
    {
      if (pluggedInterface != setup.wIndex) {
        return false;
      }
    
      uint8_t request = setup.bRequest;
      uint8_t requestType = setup.bmRequestType;
    
      if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
      {
        return true;
      }
    
      if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE) {
        if (request == HID_SET_REPORT) {
          if(setup.wValueH == HID_REPORT_TYPE_OUTPUT && setup.wLength == NUMBER_OF_LIGHTS){
            USB_RecvControl(led_data.raw, NUMBER_OF_LIGHTS);
            light_update(led_data.leds.singles, led_data.leds.rgb);
            return true;
          }
        }
      }
    
      return false;
    }
};

HIDLED_ HIDLeds;
