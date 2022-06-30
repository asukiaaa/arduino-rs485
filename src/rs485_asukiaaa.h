#ifndef _RS485_ASUKIAAA_H_
#define _RS485_ASUKIAAA_H_

#include <Arduino.h>

#define RS485_ASUKIAAA_VERSION_MAJOR 1
#define RS485_ASUKIAAA_VERSION_MINOR 2
#define RS485_ASUKIAAA_VERSION_PATCH 5

namespace rs485_asukiaaa {
namespace ModbusRtu {
enum FnCode {
  Read = 0x03,
  Write = 0x06,
  WriteMulti = 0x10,
};

// enum Exception {
//   InvalidFn = 1,
//   InvalidAddr = 2,
//   InvalidData = 3,
//   SlaveError = 4,
// };

enum Error : uint8_t {
  None = 0,
  NoResponse = 11,
  UnmatchCrc = 12,
  UnmatchAddress = 13,
  UnmatchFnCode = 14,
  ShortDataLen = 15,
};

String getStrOfError(Error e);
unsigned long getMsSilintIntervalByBaudrate(unsigned long baudrate);

class Central {
 public:
  unsigned long msSilentInterval = 16;

  Central(HardwareSerial* serial, int16_t pinDe, int16_t pinRe);
  void begin(unsigned long baudrate, unsigned long config = SERIAL_8N1);
  void beginWithoutSerial();
  void setDelayFn(void (*customDelay)(unsigned long ms));
  Error readQuery(uint8_t address, uint8_t fnCode, uint8_t* data,
                  uint16_t dataLen, unsigned long msTimeout = 50UL);
  Error readRegistersBy16t(uint8_t deviceAddress, uint16_t readStartAddress,
                           uint16_t* registerData, uint16_t dataLen);
  Error readRegistersBy32t(uint8_t deviceAddress, uint16_t readStartAddress,
                           uint32_t* registerData, uint16_t dataLen);
  void writeQuery(uint8_t address, uint8_t fnCode, uint8_t* data,
                  uint16_t dataLen);
  Error writeRegisterBy16t(uint8_t deviceAddress, uint16_t registerAddress,
                           uint16_t data16bit);
  Error writeRegistersBy16t(uint8_t deviceAddress, uint16_t registerAddress,
                            uint16_t* registerData, uint16_t dataLen);
  Error writeRegistersBy32t(uint8_t deviceAddress, uint16_t registerAddress,
                            uint32_t* registerData, uint16_t dataLen);

 private:
  HardwareSerial* serial;
  int16_t pinDe;
  int16_t pinRe;
  unsigned long lastActionAt = 0;

  void waitForSilentIntervalIfNecessary();
  void setPinDeRe(bool pinState);
  static uint16_t uint8tArrToUint16t(uint8_t* data);
  static uint32_t uint16tArrToUint32t(uint16_t* data);
  static void uint32tToUint16tArr(uint32_t v32t, uint16_t* arr);
  void (*_delay)(unsigned long ms) = [](unsigned long ms){ delay(ms); };
};

uint16_t createCRC16(const uint8_t* data, uint16_t dataLen);

}  // namespace ModbusRtu
}  // namespace rs485_asukiaaa

#endif
