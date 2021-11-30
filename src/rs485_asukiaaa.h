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
      void begin(unsigned long baudrate, unsigned long config = SERIAL_8N1);
      void beginWithoutSerial();
      int readQuery(uint8_t address, uint8_t fnCode, uint8_t* data, uint16_t dataLen, unsigned long msTimeout = 50UL);
      int readRegistersBy16t(uint8_t deviceAddress, uint16_t readStartAddress, uint16_t* registerData, uint16_t dataLen);
      int readRegistersBy32t(uint8_t deviceAddress, uint16_t readStartAddress, uint32_t* registerData, uint16_t dataLen);
      void writeQuery(uint8_t address, uint8_t fnCode, uint8_t* data, uint16_t dataLen);
      int writeRegisterBy16t(uint8_t deviceAddress, uint16_t registerAddress, uint16_t data16bit);
      int writeRegistersBy16t(uint8_t deviceAddress, uint16_t registerAddress, uint16_t* registerData, uint16_t dataLen);
      int writeRegistersBy32t(uint8_t deviceAddress, uint16_t registerAddress, uint32_t* registerData, uint16_t dataLen);
    private:
      HardwareSerial* serial;
      int16_t pinDe;
      int16_t pinRe;

      static uint16_t uint8tArrToUint16t(uint8_t *data);
      static uint32_t uint16tArrToUint32t(uint16_t *data);
      static void uint32tToUint16tArr(uint32_t v32t, uint16_t* arr);
    };

    uint16_t createCRC16(const uint8_t* data, uint16_t dataLen);
  }
}

#endif
