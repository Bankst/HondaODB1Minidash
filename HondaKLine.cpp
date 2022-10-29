#include "HondaKLine.h"
#include "Arduino.h"

HondaKLine::HondaKLine(const uint8_t klinePin)
 : m_klinePin(klinePin) {
  s_klineSerial.begin(9600);
}

void HondaKLine::init() {
  s_klineSerial.write(0x68);
  delay(k_writeDelay);
  s_klineSerial.write(0x6a);
  delay(k_writeDelay);
  s_klineSerial.write(0xf5);
  delay(k_writeDelay);
  s_klineSerial.write(0xaf);
  delay(k_writeDelay);
  s_klineSerial.write(0xbf);
  delay(k_writeDelay);
  s_klineSerial.write(0xb3);
  delay(k_writeDelay);
  s_klineSerial.write(0xb2);
  delay(k_writeDelay);
  s_klineSerial.write(0xc1);
  delay(k_writeDelay);
  s_klineSerial.write(0xdb);
  delay(k_writeDelay);
  s_klineSerial.write(0xb3);
  delay(k_writeDelay);
  s_klineSerial.write(0xe9);
  delay(k_writeDelay);
}

int HondaKLine::dlcCommand(uint8_t cmd, uint8_t num, uint8_t loc, uint8_t len) {
  uint8_t crc = (0xFF - (cmd + num + loc + len - 0x01)); // checksum FF - (cmd + num + loc + len - 0x01)
  memset(dlcData, 0, sizeof(dlcData));

  // s_klineSerial.listen(); // doesn't seem necessary

  s_klineSerial.write(cmd);  // header/cmd read memory ??
  delay(k_writeDelay);
  s_klineSerial.write(num);  // num of bytes to send
  delay(k_writeDelay);
  s_klineSerial.write(loc);  // address
  delay(k_writeDelay);
  s_klineSerial.write(len);  // num of bytes to read
  delay(k_writeDelay);
  s_klineSerial.write(crc);  // checksum
  delay(k_writeDelay);

  unsigned long timeout = millis() + 30; // 30ms timeout

  int i = 0;
  while (i < (len + 3) && millis() < timeout) {
    if (s_klineSerial.available()) {
      dlcData[i] = s_klineSerial.read();
      i++;
    }
  }

  if (i < (len + 3)) { // timeout
    dlcTimeout++;
    if (dlcTimeout > 1000) { dlcTimeout = 0; }
    return -1;
  }

  // checksum
   crc = 0;
   for (i = 0; i < len + 2; i++) {
     crc = crc + dlcData[i];
   }
   crc = 0xFF - (crc - 1);
   if (crc != dlcData[len + 2]) {
    return -2;
   }
   return 0; // success
}

// #define KLINE_DEBUG
int HondaKLine::updateEcuData() {
  int ret = 0;
  float f = 0;
#if 1
  int row1Ret = dlcCommand(0x20, 0x05, 0x00, 0x10);
  #ifdef KLINE_DEBUG
    if (row1Ret < 0) {
      // Serial.print("FAIL, ");
      // Serial.println(row1Ret == -1 ? "timeout" : "checksum");
    } else {
      Serial.print("KLINE_DEBUG(ROW1): ");
      Serial.print("{ ");
      for (uint8_t i = 0; i < sizeof(dlcData); i++) {
        Serial.print(dlcData[i]);
        if (i != (sizeof(dlcData) - 1)) {
          Serial.print(", ");
        }
      }
      Serial.println(" }");
    }
  #endif // KLINE_DEBUG

  if (row1Ret == 0) { // row 1
    // calculate RPM
    int rpm = 0;
    if (obd_select == 1) rpm = 1875000 / (dlcData[2] * 256 + dlcData[3] + 1);
    if (obd_select == 2) rpm = (dlcData[2] * 256 + dlcData[3]) / 4; // OBD2
    // in odb1 rpm is -1
    if (rpm < 0) { rpm = 0; }

    int vss = dlcData[4];
    // calculate gear
    m_ecuData.gear = vss / (rpm + 1) * 150 + 0.3;

    m_ecuData.engineRpm = rpm;
    m_ecuData.vehicleSpeedSensor = vss;
    m_ecuData.airconSwitch = bitRead(dlcData[10], 2);
    m_ecuData.vtecSwitch = bitRead(dlcData[12], 3);
  } else {
    // m_ecuData.engineRpm = 0;
    // m_ecuData.vehicleSpeedSensor = 0;
    // m_ecuData.airconSwitch = false;
    // m_ecuData.vtecSwitch = false;
    return row1Ret;
  }

  int row2Ret = dlcCommand(0x20, 0x05, 0x10, 0x10);

  #ifdef KLINE_DEBUG
    if (row2Ret < 0) {
      // Serial.print("FAIL, ");
      // Serial.println(row2Ret == -1 ? "timeout" : "checksum");
    } else {
      Serial.print("KLINE_DEBUG(ROW2): ");
      Serial.print("{ ");
      for (uint8_t i = 0; i < sizeof(dlcData); i++) {
        Serial.print(dlcData[i]);
        if (i != (sizeof(dlcData) - 1)) {
          Serial.print(", ");
        }
      }
      Serial.println(" }");
    }
  #endif // KLINE_DEBUG  

  if (row2Ret == 0) { // row 2
    // calculate engine coolant temp
    f = dlcData[2];
    if (f == 0) m_ecuData.engineCoolantTemp = 0;
    else m_ecuData.engineCoolantTemp = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;

    // calculate intake air temp
    f = dlcData[3];
    if (f == 0) m_ecuData.intakeAirTemp = 0;
    else m_ecuData.intakeAirTemp = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;

    // calculate manifold air pressure (PSI)
    m_ecuData.mapPsi = (dlcData[4] * 0.716 - 5) * 0.14504; // 101kPa @ off/WOT, 10-30kPa @ idle

    // calculate barometric pressure (PSI)
    m_ecuData.barometerPsi = (dlcData[5] * 0.716 - 5) * 0.14504;

    // calculate throttle position
    int tps = dlcData[6];
    if (tps == 0) m_ecuData.throttlePos = 0;
    else m_ecuData.throttlePos = ( - 24) / 2;

    // calculate O2 sensor voltage
    f = dlcData[7];
    m_ecuData.o2Voltage = f / 51.3;

    // calculate system voltage
    f = dlcData[9];
    m_ecuData.systemVoltage = f / 10.45;

    // calculate LMT (???)
    f = dlcData[9];
    m_ecuData.unk_lmt = (f - 24) / 4;

    // calculate
    m_ecuData.iacvDutycycle = dlcData[10] / 2.55;
  } else {
    // m_ecuData.engineCoolantTemp = 0;
    // m_ecuData.intakeAirTemp = 0;
    // m_ecuData.mapPsi = 0;
    // m_ecuData.barometerPsi = 0;
    // m_ecuData.throttlePos = 0;
    // m_ecuData.o2Voltage = 0;
    // m_ecuData.systemVoltage = 0;
    // m_ecuData.unk_lmt = 0;
    // m_ecuData.iacvDutycycle = 0;
    return row2Ret;
  }

  if (dlcCommand(0x20, 0x05, 0x30, 0x10) == 0) { // row 4
    m_ecuData.knockSensor = dlcData[14] / 51; // range 0-5;
  } else {
    // m_ecuData.knockSensor = 0;
  }
#else
  m_ecuData.engineRpm = random(500, 1500);
  m_ecuData.engineCoolantTemp = random(50, 200);
  m_ecuData.intakeAirTemp = random(30, 120);
  m_ecuData.mapPsi = random(-15, 15);
  // m_ecuData.barometerPsi = random(0, 5);
  m_ecuData.throttlePos = random(0, 100);
  m_ecuData.ignitionDeg = random(-30, 30);
  m_ecuData.systemVoltage = ( 12.0 * (random(80, 140) / 100.0) );
  m_ecuData.vehicleSpeedSensor = random(0, 255);
#endif

  // if (RPM, MAP, IAT) available, calculate IMAP/MAF
  if (m_ecuData.engineRpm != 0 && m_ecuData.mapPsi != 0 && m_ecuData.intakeAirTemp != 0) {
    // IMAP = RPM * MAP / IAT / 2
    // MAF = (IMAP/60)*(VE/100)*(Eng Disp)*(MMA)/(R)
    // Where: VE = 80% (Volumetric Efficiency), R = 8.314 J/°K/mole, MMA = 28.97 g/mole (Molecular mass of air)
    int imap = m_ecuData.engineRpm * m_ecuData.mapPsi / (m_ecuData.intakeAirTemp + 273) / 2;
    int maf = (imap / 60) * (80 / 100) * 1.595 * 28.9644 / 8.314472;
  }
  return 0;
}

void HondaKLine::scanDtcErrors() {
  uint8_t i = 0;

  if (dlcCommand(0x20, 0x05, 0x40, 0x10)) { // row 1
    for (i = 0; i < 14; i++) {
      if (dlcData[i + 2] >> 4) {
        dtcErrors[i] = i * 2;
        dtcCount++;
      }
      if (dlcData[i + 2] & 0xF) {
        dtcErrors[i] = (i * 2) + 1;

        // bodge
        if (dtcErrors[i] == 23) { dtcErrors[i] = 22; }
        if (dtcErrors[i] == 24) { dtcErrors[i] = 23; }

        dtcCount++;
      }
    }
  }
}

void HondaKLine::resetEcu() {
  dlcCommand(0x21, 0x04, 0x01, 0x00);
}
