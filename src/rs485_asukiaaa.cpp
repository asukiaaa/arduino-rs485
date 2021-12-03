#include "rs485_asukiaaa.h"

// #define DEBUG_PRINT_RS485

namespace rs485_asukiaaa {
namespace ModbusRtu {

String getStrOfError(Error e) {
  switch (e) {
    case Error::None:
      return "none";
    case Error::NoResponse:
      return "no response";
    case Error::UnmatchAddress:
      return "unmatch address";
    case Error::UnmatchCrc:
      return "unmatch crc";
    case Error::UnmatchFnCode:
      return "unmtch fn code";
    case Error::ShortDataLen:
      return "short data len";
  }
  return "";
}

Central::Central(HardwareSerial* serial, int16_t pinDe, int16_t pinRe) {
  this->serial = serial;
  this->pinDe = pinDe;
  this->pinRe = pinRe;
}

void Central::beginWithoutSerial() {
  pinMode(pinDe, OUTPUT);
  pinMode(pinRe, OUTPUT);
  setPinDeRe(LOW);
}

void Central::setPinDeRe(bool pinState) {
  digitalWrite(pinDe, pinState);
  if (pinDe != pinRe) {
    digitalWrite(pinRe, pinState);
  }
}

void Central::begin(unsigned long baudrate, unsigned long config) {
  beginWithoutSerial();
  serial->begin(baudrate, config);
}

void Central::writeQuery(uint8_t address, uint8_t fnCode, uint8_t* data,
                         uint16_t dataLen) {
  setPinDeRe(HIGH);
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
  for (i = 0; i < queryLen; ++i) {
    serial->write(queryBuffer[i]);
  }
  serial->flush();
  delay(1);
  setPinDeRe(LOW);
#ifdef DEBUG_PRINT_RS485
  Serial.print("Send: ");
  for (i = 0; i < queryLen; ++i) {
    Serial.print(queryBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
#endif
}

Error Central::writeRegisterBy16t(uint8_t deviceAddress,
                                  uint16_t registerAddress,
                                  uint16_t data16bit) {
  uint8_t data[] = {highByte(registerAddress), lowByte(registerAddress),
                    highByte(data16bit), lowByte(data16bit)};
  const int writeFnCode = rs485_asukiaaa::ModbusRtu::Write;
  writeQuery(deviceAddress, writeFnCode, data, sizeof(data));
  return readQuery(deviceAddress, writeFnCode, data, sizeof(data));
}

Error Central::writeRegistersBy32t(uint8_t deviceAddress,
                                   uint16_t writeStartAddress,
                                   uint32_t* registerData, uint16_t dataLen) {
  uint16_t data[dataLen * 2];
  for (int i = 0; i < dataLen; ++i) {
    uint32tToUint16tArr(registerData[i], &data[i * 2]);
  }
  return writeRegistersBy16t(deviceAddress, writeStartAddress, data,
                             dataLen * 2);
}

Error Central::writeRegistersBy16t(uint8_t deviceAddress,
                                   uint16_t registerAddress,
                                   uint16_t* registerData, uint16_t dataLen) {
  const uint16_t writeLen = dataLen * 2 + 5;
  uint8_t writeData[writeLen] = {
      highByte(registerAddress), lowByte(registerAddress), highByte(dataLen),
      lowByte(dataLen),          (uint8_t)(dataLen * 2),
  };
  for (int i = 0; i < dataLen; ++i) {
    writeData[i * 2 + 5] = highByte(registerData[i]);
    writeData[i * 2 + 6] = lowByte(registerData[i]);
  }
  const int writeMultiFnCode = rs485_asukiaaa::ModbusRtu::WriteMulti;
  writeQuery(deviceAddress, writeMultiFnCode, writeData, writeLen);
  uint8_t readData[4];
  return readQuery(deviceAddress, writeMultiFnCode, readData, sizeof(readData));
  // Parse readData if cannot communicate as expected
}

Error Central::readQuery(uint8_t address, uint8_t fnCode, uint8_t* data,
                         uint16_t dataLen, unsigned long msTimeout) {
  uint16_t queryIndex = 0;
  uint16_t queryLenToReceive = dataLen + 4;
  uint8_t queryBuffer[queryLenToReceive];
  unsigned long waitFrom = millis();
#ifdef DEBUG_PRINT_RS485
  Serial.print("Receive: ");
#endif
  while (serial->available() > 0 || millis() - waitFrom < msTimeout) {
    if (queryIndex >= queryLenToReceive) {
      break;
    }
    if (serial->available() == 0) {
      delay(1);
      continue;
    }
    waitFrom = millis();
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
  if (dataLen < queryIndex - 4) {
    return Error::ShortDataLen;
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

  for (uint16_t i = 0; i < dataLen; ++i) {
    data[i] = queryBuffer[i + 2];
  }
  return Error::None;
}

Error Central::readRegistersBy32t(uint8_t deviceAddress,
                                  uint16_t readStartAddress,
                                  uint32_t* registerData, uint16_t dataLen) {
  uint16_t buffs[dataLen * 2];
  auto result =
      readRegistersBy16t(deviceAddress, readStartAddress, buffs, dataLen * 2);
  if (result != 0) {
    return result;
  }
  for (uint16_t i = 0; i < dataLen; ++i) {
    registerData[i] = uint16tArrToUint32t(&buffs[i * 2]);
  }
  return result;
}

Error Central::readRegistersBy16t(uint8_t deviceAddress,
                                  uint16_t readStartAddress,
                                  uint16_t* registerData, uint16_t dataLen) {
  uint8_t data[] = {highByte(readStartAddress), lowByte(readStartAddress),
                    highByte(dataLen), lowByte(dataLen)};
  auto readFnCode = rs485_asukiaaa::ModbusRtu::FnCode::Read;
  writeQuery(deviceAddress, readFnCode, data, sizeof(data));
  const int buffLen = dataLen * 2 + 1;
  uint8_t uint8Buffs[buffLen];
  auto result = readQuery(deviceAddress, readFnCode, uint8Buffs, buffLen);
  if (result != 0) {
    return result;
  }
  for (uint16_t i = 0; i < dataLen; ++i) {
    registerData[i] = uint8tArrToUint16t(
        &uint8Buffs[i * 2 + 1]);  // + 1 to skip data length byte
  }
  return result;
}

uint16_t Central::uint8tArrToUint16t(uint8_t* data) {
  return ((uint16_t)data[0]) << 8 | (uint16_t)data[1];
}

uint32_t Central::uint16tArrToUint32t(uint16_t* data) {
  return ((uint32_t)data[0]) << 16 | (uint32_t)data[1];
}

void Central::uint32tToUint16tArr(uint32_t v32t, uint16_t* arr) {
  arr[0] = v32t >> 16;
  arr[1] = v32t & 0xffff;
}

uint16_t createCRC16(const uint8_t* data, uint16_t dataLen) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < dataLen; pos++) {
    crc ^= (uint16_t)data[pos];
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

}  // namespace ModbusRtu
}  // namespace rs485_asukiaaa
