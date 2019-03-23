# Mbed.Modbus
Library modbus for mbed framework

Example with STM32F4:

#include <mbed.h>
#include "ModbusSerial.h"

Serial mbPort(PA_9, PA_10, 9600); //Using UART1 for modbus port
ModbusSerial mb;

int main() {

    // Config Modbus Serial (port, speed, byte format) 
    SerialFormat format;
    format.bits = 8;
    format.parity = SerialBase::None;
    format.stop_bits = 1;
    mb.config(&mbPort, 19200, format);
    
    // Set the Slave ID (1-247)
    mb.setSlaveId(1);
    
    //Add Holding Reg
    mb.addHreg(0, 0);
    
    uint16_t test = 0;

  while(1) {
   mb.Hreg(0, test);
   test++;
   wait_ms(100);    //unnecessary
  }
}
