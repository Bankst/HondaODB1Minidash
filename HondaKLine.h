#include <SingleWireSerial.h>

static SingleWireSerial s_klineSerial(false);

typedef struct {
  int rpm;
  int spd;
  float bat;
  int ect;
  int iat;
  int map;
  int tps;
  int ign;
} DispEcuInfo_t;

typedef struct {
  int engineRpm;
  int engineCoolantTemp;
  int intakeAirTemp;
  int mapPsi;
  int barometerPsi;
  int throttlePos; // percent
  int shortTermFuelTrim;
  int longTermFuelTrim;
  int injectorMillis;
  int ignitionDeg;
  int unk_lmt;
  int iacvDutycycle;
  int knockSensor;
  float systemVoltage;
  float o2Voltage;
  uint8_t vehicleSpeedSensor;
  bool airconSwitch;
  bool brakeSwitch;
  bool vtecSwitch;

  // extrapolated
  uint8_t gear;
  int massAirFlow;
} BasicEcuData_t;

typedef struct {
  int peakRpm;
  int peakVoltage;
  int peakMap;
  int peakThrottle;
  int peakEct;
  int peakIat;
  uint8_t peakVehicleSpeed;
  uint8_t avgVehicleSpeed;
} ExtraEcuData_t;

class HondaKLine {
public:
  HondaKLine(const uint8_t klinePin);

  void init();
  int updateEcuData();
  void scanDtcErrors();
  void resetEcu();

  BasicEcuData_t * getEcuData() { return &m_ecuData; }
  ExtraEcuData_t * getExtraData() { return &m_extraData; }

private: // methods
  int dlcCommand(uint8_t cmd, uint8_t num, uint8_t loc, uint8_t len);
private: // variables
  const uint8_t k_writeDelay = 1;

  const uint8_t m_klinePin;
  // SoftwareSerialWithHalfDuplex *m_klineSerial;

  // ecu data
  BasicEcuData_t m_ecuData;
  ExtraEcuData_t m_extraData;

  // calculated values
  // int rpmtop=0,volttop=0,mapstop=0,tpstop=0,ecttop=0,iattop=0;
  // unsigned long vsssum=0,running_time=0,idle_time=0,distance=0;
  // uint8_t vsstop=0,vssavg=0;

  const uint8_t obd_select = 1;

  uint8_t dlcData[20] = {0}; // dlc data buffer
  unsigned long dlcTimeout = 0, dlcChecksumError = 0;

  int dtcErrors[14], dtcCount = 0;
};
