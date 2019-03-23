/*
    main.cpp - Main file test modbus
    Copyright (C) 2019 Khiem Tran
*/
#include <mbed.h>
#include "ModbusSerial.h"

Serial mbPort(PA_9, PA_10, 9600);
//Serial pc(PC_6, PC_7, 9600);
ModbusSerial mb;

int main() {

      // Config Modbus Serial (port, speed, byte format) 
    SerialFormat format;
    format.bits = 8;
    format.parity = SerialBase::None;
    format.stop_bits = 1;

    //Config Modbus
    mb.config(&mbPort, 19200, format);
    // Set the Slave ID (1-247)
    mb.setSlaveId(1);
    
    //Add Holding Reg
    mb.addHreg(0, 0);
 //   pc.printf("PC!\n");
    
    uint16_t test = 0;

  while(1) {
   mb.Hreg(0, test);
   test++;
   wait_ms(100);    //unnecessary
  }
}
