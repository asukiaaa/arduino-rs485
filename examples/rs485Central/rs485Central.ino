#include <rs485_asukiaaa.h>

#define PIN_DE 5
#define PIN_RE 6
#define BAUDRATE 115200
rs485_asukiaaa::Modbus modbus(&Serial1, PIN_DE, PIN_RE);

void setup() {
  modbus.begin(BAUDRATE);
}

void loop() {
}
