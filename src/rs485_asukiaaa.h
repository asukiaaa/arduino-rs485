#ifndef _RS485_ASUKIAAA_H_
#define _RS485_ASUKIAAA_H_

#include <Arduino.h>

namespace rs485_asukiaaa {
  enum ModbusError { NoResponse = 11,
                     UnmatchCrc = 12,
                     UnmatchAddress = 13,
                     UnmatchFnCode = 14,
                     UnmatchDataLen = 15,
                     OverQueryMaxLen = 16,
  };

  class Modbus {
   public:
    Modbus(HardwareSerial* serial, int16_t pinDe, int16_t pinRe);
    void begin(long baudrate);
    void beginWithoutSerial();
    void writeQuery(uint8_t address, uint8_t fnCode, uint8_t* data, uint16_t dataLen);
    uint8_t readQuery(uint8_t address, uint8_t fnCode, uint8_t* data, uint16_t dataLen, unsigned long msTimeout = 50UL);
   private:
    HardwareSerial* serial;
    int16_t pinDe;
    int16_t pinRe;
  };

  uint16_t createCRC16(const uint8_t* data, uint16_t dataLen);
}

#endif
