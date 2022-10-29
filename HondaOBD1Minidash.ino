#define KLINE_DEBUG TRUE

#include "HondaKLine.h"
#include "SSD1306_minimal.h"
#include "HT16K33.h"

HondaKLine ecu{12}; // OBD1 K-Line on pin 12
SSD1306_Mini oled{0x3D};
HT16K33 seg{0x70};

DispEcuInfo_t dispInfo;

int lcd_putc( char c, FILE * )
{
  oled.printChar(c);
  return 0;
}

uint8_t oledDispState = 0;

unsigned long last7SegUpdateMs = 0;
unsigned long lastVtecBlinkMs = 0;
bool vtecBlinkState = false;

unsigned long lastOledUpdateMs = 0;
unsigned long oledStateEnterMs = 0;
unsigned long oledGenTimer1 = 0;

static const char hdrBar = '='; // 0x07

static const char kHeaderStr[18] = {
  0x10, hdrBar, hdrBar, hdrBar, hdrBar,
  'L', 'u', 'd', 'e', 'D', 'a', 's', 'h',
  hdrBar, hdrBar, hdrBar, hdrBar, 0x11
};

void render7SegDash() {
  int rpm = dispInfo.rpm;
  bool vtecOn = rpm > 5000;
  
  if(vtecOn && millis() - lastVtecBlinkMs > 100) {
    vtecBlinkState = !vtecBlinkState;
    lastVtecBlinkMs = millis();
    seg.displayExtraLeds(vtecBlinkState ? 0x02 : 0x00);
  } else if(!vtecOn && vtecBlinkState) {
    seg.displayExtraLeds(0x00);
    vtecBlinkState = false;
  }
  if (rpm > 9999) rpm = 9999;
  seg.displayInt(rpm);
}

int calcDisplayInfo(BasicEcuData_t * ecuData) {
  dispInfo.rpm = ecuData->engineRpm;
  dispInfo.spd = ecuData->vehicleSpeedSensor;

  // float volts = ecuData->systemVoltage * 10;
  // float dec = volts % 10.0f;
  // dispInfo->batInt = volts / 10;
  // dispInfo->batDec = dispInfo->batInt % 10;
  dispInfo.bat = ecuData->systemVoltage;

  dispInfo.ect = ecuData->engineCoolantTemp;
  dispInfo.iat = ecuData->intakeAirTemp;
  dispInfo.map = ecuData->mapPsi;
  dispInfo.tps = ecuData->throttlePos;
  dispInfo.ign = ecuData->ignitionDeg;

  return 0;
}

void renderOledInit() {
  static const uint8_t dotWidth = 4;
  static uint8_t numDots = 0;

  oled.cursorTo(0, 3);
  oled.printString(kHeaderStr);
  oled.cursorTo(0, 4);
  oled.printString("   Initializing");

  if (millis() - oledGenTimer1 > 125) {
    // render dots and spaces
    for (uint8_t i = 0; i < dotWidth; i++) {
      if (i < numDots) { oled.printChar('.'); }
      else { oled.printChar(' '); }
    }
    numDots++;
    if (numDots > 3) { numDots = 0; }
    oledGenTimer1 = millis();
  }
}

void renderOledDash() {
  oled.cursorTo(0, 0);
  oled.printString(kHeaderStr);
  oled.cursorTo(0, 1);
  printf("RPM:%4d", dispInfo.rpm);
  printf("  SPD: %3d", dispInfo.spd);
  oled.cursorTo(0, 3);

  static char batStr[5];
  dtostrf(dispInfo.bat, 4, 1, batStr);
  printf("BAT:%sV", batStr);
  // printf("BAT:%2d.%dV", dispInfo.batInt, dispInfo.batDec);

  printf(" ECT:%3d", dispInfo.ect); oled.printChar(0xf8);
  oled.cursorTo(0, 5);
  printf("IAT:%3d", dispInfo.iat); oled.printChar(0xf8); // 8chars
  printf("  MAP:%4d", dispInfo.map);
  oled.cursorTo(0, 7);
  printf("TPS:%3d", dispInfo.tps); oled.printChar(0x25); // printf doesn't like %
  printf("  IGN:%3d", dispInfo.ign); oled.printChar(0xf8);
}

void oledDisplayHandler() {
  if (oledDispState == 0) {
    renderOledInit();

    if (millis() - oledStateEnterMs > 1000) {
      oledDispState = 1;
      oledStateEnterMs = millis();
      oled.clear();
    }
  } else if (oledDispState == 1) {
    oled.cursorTo(0, 3);
    oled.printString(kHeaderStr);
    oled.cursorTo(0, 4);
    oled.printString("      Init OK!    ");

    if (millis() - oledStateEnterMs > 300) {
      oledDispState = 2;
      oledStateEnterMs = millis();
      oled.clear();
    }
  } else if (oledDispState == 2) {
    renderOledDash();
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Init OK!");
  randomSeed(analogRead(0));

  oled.init();
  oled.clear();
  oled.cursorTo(0, 0);
  fdevopen(&lcd_putc, NULL);

  // Wire.setClock(400000);

  seg.begin();
  seg.displayClear();
  seg.displayOn();
  seg.brightness(2);
  seg.blink(0);
  seg.cacheOn();

  delay(100);

  ecu.init();
  Serial.println("ECU Init Sent");

  lastOledUpdateMs = millis();
  oledStateEnterMs = lastOledUpdateMs;

  // uint8_t charsPrinted = 0;
  // uint8_t row = 0;
  // for (uint8_t i = 0x00; i < 0xff; i++) {
  //   if (row > 7) continue;
  //   oled.printChar(i);
  //   if (++charsPrinted >= 18) {
  //     charsPrinted = 0;
  //     oled.cursorTo(0, ++row);
  //   }
  // }

  // oled.printString("IAT: 50 °");
}

unsigned long lastEcuUpdateMs = 0;
bool lastReadFailed = false;

void loop() {
  if (oledDispState == 2) {
    // if (millis() - lastEcuUpdateMs > lastReadFailed ? 1000 : 50) {
    if (millis() - lastEcuUpdateMs > 50) {
      int ecuRet = 0;
      ecuRet = ecu.updateEcuData();
      // if (ecuRet != 0) {
      //   Serial.print("ECU get failed, reason: ");
      //   Serial.println(ecuRet == -1 ? "timeout" : "checksum");
      //   lastReadFailed = true;
      // } else {
      //   Serial.println("ECU Get OK");
      //   lastReadFailed = false;
      // }

      // if (lastReadFailed) {
      //   ecu.init();
      //   Serial.println("ECU Re-Init!!!");
      // }
    }
    calcDisplayInfo(ecu.getEcuData());
  }

  oledDisplayHandler();
  render7SegDash();
}
