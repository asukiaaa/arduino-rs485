#ifndef _RS485_ASUKIAAA_H_
#define _RS485_ASUKIAAA_H_

#include <Arduino.h>

namespace rs485_asukiaaa {
  namespace ModbusRtu {
    enum FnCode { Read       = 0x03,
                  Write      = 0x06,
                  WriteMulti = 0x10,
    };

    enum Error { NoResponse = 11,
                 UnmatchCrc = 12,
                 UnmatchAddress = 13,
                 UnmatchFnCode = 14,
                 UnmatchDataLen = 15,
                 OverQueryMaxLen = 16,
    };

    class Central {
    public:
      Central(HardwareSerial* serial, int16_t pinDe, int16_t pinRe);
      void begin(long baudrate, int config = SERIAL_8N1);
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
}

#endif
