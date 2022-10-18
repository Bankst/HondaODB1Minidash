#include "HondaKLine.h"
#include "SSD1306_minimal.h"

HondaKLine ecu{12}; // OBD1 K-Line on pin 12
SSD1306_Mini oled{0x3D};

DispEcuInfo_t dispInfo;

int lcd_putc( char c, FILE * )
{
  oled.printChar(c);
  return 0;
}

uint8_t dispState = 0;

unsigned long lastDisplayUpdate = 0;
unsigned long displayStateEnterMs = 0;
unsigned long displayGenTimer1 = 0;

static const char hdrBar = '='; // 0x07

static const char kHeaderStr[18] = {
  0x10, hdrBar, hdrBar, hdrBar, hdrBar,
  'L', 'u', 'd', 'e', 'D', 'a', 's', 'h',
  hdrBar, hdrBar, hdrBar, hdrBar, 0x11
};

int calcDisplayInfo(DispEcuInfo_t * dispInfo, BasicEcuData_t * ecuData) {
  dispInfo->rpm = ecuData->engineRpm;
  dispInfo->spd = ecuData->vehicleSpeedSensor;

  // float volts = ecuData->systemVoltage * 10;
  // float dec = volts % 10.0f;
  // dispInfo->batInt = volts / 10;
  // dispInfo->batDec = dispInfo->batInt % 10;
  dispInfo->bat = ecuData->systemVoltage;

  dispInfo->ect = ecuData->engineCoolantTemp;
  dispInfo->iat = ecuData->intakeAirTemp;
  dispInfo->map = ecuData->mapPsi;
  dispInfo->tps = ecuData->throttlePos;
  dispInfo->ign = ecuData->ignitionDeg;

  return 0;
}

void renderInit() {
  static const uint8_t dotWidth = 4;
  static uint8_t numDots = 0;

  oled.cursorTo(0, 3);
  oled.printString(kHeaderStr);
  oled.cursorTo(0, 4);
  oled.printString("   Initializing");

  if (millis() - displayGenTimer1 > 125) {
    // render dots and spaces
    for (uint8_t i = 0; i < dotWidth; i++) {
      if (i < numDots) { oled.printChar('.'); }
      else { oled.printChar(' '); }
    }
    numDots++;
    if (numDots > 3) { numDots = 0; }
    displayGenTimer1 = millis();
  }
}

void renderDash() {
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

void displayHandler() {
  if (dispState == 0) {
    renderInit();

    if (millis() - displayStateEnterMs > 1000) {
      dispState = 1;
      displayStateEnterMs = millis();
      oled.clear();
    }
  } else if (dispState == 1) {
    oled.cursorTo(0, 3);
    oled.printString(kHeaderStr);
    oled.cursorTo(0, 4);
    oled.printString("      Init OK!    ");

    if (millis() - displayStateEnterMs > 300) {
      dispState = 2;
      displayStateEnterMs = millis();
      oled.clear();
    }
  } else if (dispState == 2) {
    renderDash();
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

  ecu.init();

  lastDisplayUpdate = millis();
  displayStateEnterMs = lastDisplayUpdate;

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

void loop() {
  if (dispState == 2) {
    ecu.updateEcuData();
    calcDisplayInfo(&dispInfo, &ecu.getEcuData());
  }

  // if (millis() - lastDisplayUpdate > 16) { // ~60fps
  displayHandler();
  // }
}
