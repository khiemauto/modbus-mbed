/*
    ModbusSerial.cpp - Source for Modbus Serial Library
    Copyright (C) 2014 AndrĂ© Sarmento Barbosa
*/
#include "ModbusSerial.h"

ModbusSerial::ModbusSerial() {

}

bool ModbusSerial::setSlaveId(uint8_t slaveId){
    _slaveId = slaveId;
    return true;
}

uint8_t ModbusSerial::getSlaveId() {
    return _slaveId;
}

bool ModbusSerial::config(Serial* port, long baud, SerialFormat format, int txPin) {
    this->_port = port;

    (*port).baud(baud);
    (*port).format(format.bits, format.parity, format.stop_bits);

    wait_ms(2000);

    if (txPin >= 0) {
        if (_txPin) delete _txPin;
        _txPin = new DigitalOut((PinName)txPin);
        _txPin->write(0);
    }

    if (baud > 19200) {
        _t15 = 750;
        _t35 = 1750;
    } else {
        _t15 = 15000000/baud; // 1T * 1.5 = T1.5
        _t35 = 35000000/baud; // 1T * 3.5 = T3.5
    }

    (*port).attach(this, &ModbusSerial::interruptRx, Serial::RxIrq);

    return true;
}

bool ModbusSerial::receive(uint8_t* frame) {
    //first byte of frame = address
    uint8_t address = frame[0];
    //Last two bytes = crc
    u_int crc = ((frame[_len - 2] << 8) | frame[_len - 1]);

    //Slave Check
    if (address != 0xFF && address != this->getSlaveId()) {
		return false;
	}

    //CRC Check
    if (crc != this->calcCrc(_frame[0], _frame+1, _len-3)) {
		return false;
    }

    //PDU starts after first byte
    //framesize PDU = framesize - address(1) - crc(2)
    this->receivePDU(frame+1);
    //No reply to Broadcasts
    if (address == 0xFF) _reply = MB_REPLY_OFF;
    return true;
}

void ModbusSerial::send(uint8_t* frame) {
    uint8_t i;

    if (this->_txPin) {
        this->_txPin->write(1);
        wait_ms(1);
    }

    for (i = 0 ; i < _len ; i++) {
        (*_port).putc(frame[i]);
    }

    (*_port).fsync();
    wait_us(_t35);

    if (this->_txPin) {
        this->_txPin->write(0);
    }
}

void ModbusSerial::sendPDU(uint8_t* pduframe) {
    if (this->_txPin) {
        this->_txPin->write(1);
        wait_ms(1);
    }

    //Send slaveId
    (*_port).putc(_slaveId);

    //Send PDU
    uint8_t i;
    for (i = 0 ; i < _len ; i++) {
        (*_port).putc(pduframe[i]);
    }

    //Send CRC
    uint16_t crc = calcCrc(_slaveId, _frame, _len);
    (*_port).putc(crc >> 8);
    (*_port).putc(crc & 0xFF);

    (*_port).fsync();
    wait_us(_t35);

    if (this->_txPin >= 0) {
        this->_txPin->write(0);
    }
}

uint16_t ModbusSerial::calcCrc(uint8_t address, uint8_t* pduFrame, uint8_t pduLen) {
	uint8_t CRCHi = 0xFF, CRCLo = 0x0FF, Index;

    Index = CRCHi ^ address;
    CRCHi = CRCLo ^ _auchCRCHi[Index];
    CRCLo = _auchCRCLo[Index];

    while (pduLen--) {
        Index = CRCHi ^ *pduFrame++;
        CRCHi = CRCLo ^ _auchCRCHi[Index];
        CRCLo = _auchCRCLo[Index];
    }

    return (CRCHi << 8) | CRCLo;
}


void ModbusSerial::interruptRx() {
    while ((*_port).readable()) {
        _rxbuffer[_rxlen] = (*_port).getc();
        _rxlen = (_rxlen + 1) % MODBUS_BUFFER_SIZE;
    }
    _request.attach_us(this, &ModbusSerial::responeRequest, _t35);
    return;
}

void ModbusSerial::responeRequest() {
    _len = _rxlen;

    if (_len == 0) return;
    
    _frame = new uint8_t[_len];
    memcpy(_frame, _rxbuffer, _len);

    if (this->receive(_frame)) {
        if (_reply == MB_REPLY_NORMAL)
            this->sendPDU(_frame);
        else
        if (_reply == MB_REPLY_ECHO)
            this->send(_frame);
    }

    delete[] _frame;
    _len = 0;
    _rxlen = 0;
}