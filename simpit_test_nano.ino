#include <Bounce2.h>
#include <KerbalSimpitMessageTypes.h>
#include <PayloadStructs.h>
#include <KerbalSimpit.h>
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define stagePin 2
#define abortPin 3
#define ag1Pin 4
#define ag2Pin 5
#define ag3Pin 6
#define ag4Pin 7
#define ag5Pin 8
#define brakesPin 9
#define gearPin 10
#define lightsPin 11
#define solarPanelsPin 12
#define throttlePin A0
#define HorizontalTranslationPin A1
#define VerticalTranslationPin A2
#define PitchPin A3
#define YawPin A4
#define parachutesPin A5
#define NUM_INPUTS sizeof(digitalInputPins)

byte digitalInputPins[] = {
  stagePin,
  abortPin,
  ag1Pin,
  ag2Pin,
  ag3Pin,
  ag4Pin,
  ag5Pin,
  brakesPin,
  gearPin,
  lightsPin,
  solarPanelsPin,
  parachutesPin
};

int16_t throttleValue = 0; //0 is minimum, 32767 is max.
int16_t sasMode = 0;

rotationMessage rotation;

Bounce inputs[NUM_INPUTS];

// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial);

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C

void setup() {
  // Open the serial connection.
  Serial.begin(115200);
  u8g2.begin();

  for (byte i = 0; i < NUM_INPUTS; i++) {
    pinMode(digitalInputPins[i], INPUT_PULLUP);
    inputs[i].attach(digitalInputPins[i]);
    inputs[i].interval(25);
  }

  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!mySimpit.init()) {
    delay(100);
  }
  // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);
  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  mySimpit.inboundHandler(messageHandler);
  // Send a message to the plugin registering for the Altitude channel.
  // The plugin will now regularly send Altitude messages while the
  // flight scene is active in-game.
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  mySimpit.registerChannel(AGACTIVATE_MESSAGE);
  mySimpit.registerChannel(AGDEACTIVATE_MESSAGE);
  mySimpit.registerChannel(CAGACTIVATE_MESSAGE);
  mySimpit.registerChannel(CAGDEACTIVATE_MESSAGE);
  mySimpit.registerChannel(THROTTLE_MESSAGE);
  mySimpit.registerChannel(ROTATION_MESSAGE);
  mySimpit.registerChannel(SASMODE_MESSAGE);
}

void loop() {
  for (byte i = 0; i < NUM_INPUTS; i++) {
    if (inputs[i].update()) {
      if (inputs[i].fell()) {
        switch (digitalInputPins[i]) {
          case stagePin :
            mySimpit.activateAction(STAGE_ACTION);
            break;
          case abortPin :
            mySimpit.activateAction(ABORT_ACTION);
            break;
          case ag1Pin : 
            mySimpit.send(SASMODE_MESSAGE, sasMode);
            sasMode++;
            if (sasMode > 9) {
              sasMode = 0;
            }
            //mySimpit.activateCAG(3);
            break;
          case ag2Pin : 
            mySimpit.activateCAG(4);
            break;
          case ag3Pin : 
            mySimpit.activateCAG(5);
            break;
          case ag4Pin : 
            mySimpit.activateCAG(6);
            break;
          case ag5Pin : 
            mySimpit.activateCAG(7);
            break;
          case brakesPin : 
            mySimpit.activateAction(BRAKES_ACTION);
            break;
          case gearPin :
            mySimpit.activateAction(GEAR_ACTION);
            break;
          case lightsPin :
            mySimpit.activateAction(LIGHT_ACTION);
            break;
          case solarPanelsPin :
            mySimpit.activateCAG(1);
            break;
          case parachutesPin :
            mySimpit.activateCAG(2);
            break;
        }
      }
      else if (inputs[i].rose()) {
        switch (digitalInputPins[i]) {
          case brakesPin : 
            mySimpit.deactivateAction(BRAKES_ACTION);
            break;
          case gearPin :
            mySimpit.deactivateAction(GEAR_ACTION);
            break;
          case lightsPin :
            mySimpit.deactivateAction(LIGHT_ACTION);
            break;
          case solarPanelsPin :
            mySimpit.deactivateCAG(1);
            break;
        }
      }
    }
  }

  throttleValue = analogRead(throttlePin);
  throttleValue = map(throttleValue, 0, 1023, 32767, 0);

  mySimpit.send(THROTTLE_MESSAGE, (unsigned char*) &throttleValue, 2);

  rotation.pitch = analogRead(PitchPin);
  if (rotation.pitch < 450 || rotation.pitch > 550) {
    rotation.pitch = map(rotation.pitch, 0, 1023, -32767, 32767);
    rotation.mask |= 1;
  } else {
    rotation.mask = modifyBit(rotation.mask, 0, 0);
  }

  rotation.yaw = analogRead(YawPin);
  if (rotation.yaw < 450 || rotation.yaw > 550) {
    rotation.yaw = map(rotation.yaw, 0, 1023, -32767, 32767);
    rotation.mask |= 4;
  } else {
    rotation.mask = modifyBit(rotation.mask, 2, 0);
  }

  /** The mask indicates which elements are intentionally set. Unset elements
      should be ignored. It should be one or more of:

      - 1: pitch
      - 2: roll
      - 4: yaw
  */
  rotation.mask = (byte)5;
  mySimpit.send(ROTATION_MESSAGE, rotation);

  u8g2.firstPage();
  do {
    compactDisplay(1.0, 1.0, 1.0, 1.0 ,1.0);
  } while (u8g2.nextPage());
  
  // Check for new serial messages.
  mySimpit.update();
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch(messageType) {
  case ALTITUDE_MESSAGE:
    // Checking if the message is the size we expect is a very basic
    // way to confirm if the message was received properly.
//    if (msgSize == sizeof(altitudeMessage)) {
//      // Create a new Altitude struct
//      altitudeMessage myAltitude;
//      // Convert the message we received to an Altitude struct.
//      myAltitude = parseAltitude(msg);
//      // Turn the LED on if the vessel is higher than 500 metres
//      // above sea level. Otherwise turn it off.
//      if (myAltitude.sealevel > 500) {
//        digitalWrite(LED_BUILTIN, HIGH);
//      } else {
//        digitalWrite(LED_BUILTIN, LOW);
//      }
//    }
    break;
  }
}

byte modifyBit(byte number, byte pos, byte value) {
  byte mask = 1 << pos;

  return (number & ~mask) | ((value << pos) & mask);
}

void compactDisplay(float apo, float peri, float lf, float sf, float vel) {
  byte tempPosition = 0;
  u8g2.setFont(u8g2_font_micro_mr);
  u8g2.setFontPosTop();
    tempPosition = u8g2.drawStr(0, 0, "Apoapsis: ");
    u8g2.setCursor(tempPosition, 0);
    u8g2.print(apo, 0);
    
    tempPosition = u8g2.drawStr(0, 6, "Periapsis: ");
    u8g2.setCursor(tempPosition, 6);
    u8g2.print(peri, 0);
    
    tempPosition = u8g2.drawStr(0, 12, "Liquid fuel: ");
    u8g2.setCursor(tempPosition, 12);
    u8g2.print(lf, 0);
    
    tempPosition = u8g2.drawStr(0, 18, "Solid fuel: ");
    u8g2.setCursor(tempPosition, 18);
    u8g2.print(sf, 0);
    
    tempPosition = u8g2.drawStr(0, 24, "Velocity: ");
    u8g2.setCursor(tempPosition, 24);
    u8g2.print(vel, 0);
}
