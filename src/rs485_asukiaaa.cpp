#include "rs485_asukiaaa.h"

// #define DEBUG_PRINT_RS485

namespace rs485_asukiaaa {
  namespace ModbusRtu {
    Central::Central(HardwareSerial* serial, int16_t pinDe, int16_t pinRe) {
      this->serial = serial;
      this->pinDe = pinDe;
      this->pinRe = pinRe;
    }

    void Central::beginWithoutSerial() {
      pinMode(pinDe, OUTPUT);
      pinMode(pinRe, OUTPUT);
      digitalWrite(pinDe, LOW);
      digitalWrite(pinRe, LOW);
    }

    void Central::begin(long baudrate) {
      beginWithoutSerial();
      serial->begin(baudrate);
    }

    void Central::writeQuery(uint8_t address, uint8_t fnCode, uint8_t* data, uint16_t dataLen) {
      digitalWrite(pinDe, HIGH);
      digitalWrite(pinRe, HIGH);
      delay(1);
      uint16_t queryLen = 4 + dataLen;
      uint16_t i;
      uint8_t queryBuffer[queryLen];
      queryBuffer[0] = address;
      queryBuffer[1] = fnCode;
      for (i = 0; i < dataLen; ++i) {
        queryBuffer[i + 2] = data[i];
      }

      uint16_t crc16 = createCRC16(queryBuffer, queryLen - 2);
      queryBuffer[queryLen - 2] = lowByte(crc16);
      queryBuffer[queryLen - 1] = highByte(crc16);

      while (serial->available()) {
        serial->read();
      }  // remove received buffer before sending
  #ifdef DEBUG_PRINT_RS485
      Serial.print("Send: ");
  #endif
      for (i = 0; i < queryLen; ++i) {
        serial->write(queryBuffer[i]);
  #ifdef DEBUG_PRINT_RS485
        Serial.print(queryBuffer[i], HEX);
        Serial.print(" ");
  #endif
      }
      serial->flush();
  #ifdef DEBUG_PRINT_RS485
      Serial.println("");
  #endif
      delay(1);
      digitalWrite(pinDe, LOW);
      digitalWrite(pinRe, LOW);
      delay(1);
    }

    uint8_t Central::readQuery(uint8_t address, uint8_t fnCode, uint8_t* data, uint16_t dataLen, unsigned long msTimeout) {
      uint16_t queryIndex = 0;
      uint16_t queryLenToReceive = dataLen + 4;
      uint8_t queryBuffer[queryLenToReceive];
      unsigned long waitFrom = millis();
  #ifdef DEBUG_PRINT_RS485
      Serial.print("Receive: ");
  #endif
      while (serial->available() || millis() - waitFrom < msTimeout) {
        if (!serial->available()) {
          delay(1);
          continue;
        }
        waitFrom = millis();
        if (queryIndex == queryLenToReceive) {
  #ifdef DEBUG_PRINT_RS485
          Serial.println("stop receiving because buffer length was over");
  #endif
          return Error::OverQueryMaxLen;
        }
        queryBuffer[queryIndex] = serial->read();
  #ifdef DEBUG_PRINT_RS485
        Serial.print(queryBuffer[queryIndex], HEX);
        Serial.print(" ");
  #endif
        ++queryIndex;
      }
  #ifdef DEBUG_PRINT_RS485
      Serial.println("");
  #endif
      if (queryIndex == 0) {
        return Error::NoResponse;
      }
      uint16_t crc = createCRC16(queryBuffer, queryIndex - 2);
      if (highByte(crc) != queryBuffer[queryIndex - 1] ||
          lowByte(crc) != queryBuffer[queryIndex - 2]) {
        return Error::UnmatchCrc;
      }
      if (queryBuffer[0] != address) {
        return Error::UnmatchAddress;
      }
      if (queryBuffer[1] != fnCode) {
        return Error::UnmatchFnCode;
      }
      if (dataLen != queryIndex - 4) {
        return Error::UnmatchDataLen;
      }

      for (uint16_t i = 0; i < dataLen; ++i) {
        data[i] = queryBuffer[i + 2];
      }
      return 0;
    }

    uint16_t createCRC16(const uint8_t* data, uint16_t dataLen) {
      uint16_t crc = 0xFFFF;
      for (uint16_t pos = 0; pos < dataLen; pos++) {
        crc ^= (uint16_t) data[pos];
        for (int i = 8; i != 0; i--) {
          if ((crc & 0x0001) != 0) {
            crc >>= 1;
            crc ^= 0xA001;
          } else {
            crc >>= 1;
          }
        }
      }
      return crc;
    }
  }
}
