/*

Copyright (c) 2020 Maxz Maxzerker (for PZEM-017)
Copyright (c) 2019 Jakub Mandula

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


/*
 * PZEM-004Tv30.h
 *
 * Interface library for the upgraded version of PZEM-004T v3.0
 * Based on the PZEM004T library by @olehs https://github.com/olehs/PZEM004T
 *
 * Author: Jakub Mandula https://github.com/mandulaj
 *
 *
*/

/*
 * PZEM-017Tv1.h
 *
 * Interface library for PZEM-017 v1.0
 * Based on the PZEM004T library by Jakub Mandula https://github.com/mandulaj
 *
 * Author: Maxz Maxzerker https://github.com/maxzerker
 *
 *
*/


#ifndef PZEM017v1_H
#define PZEM017v1_H



#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// #define PZEM004_NO_SWSERIAL
#if (not defined(PZEM004_NO_SWSERIAL)) && (defined(__AVR__) || defined(ESP8266) && (not defined(ESP32)))
#define PZEM004_SOFTSERIAL
#endif

#if defined(PZEM004_SOFTSERIAL)
#include <SoftwareSerial.h>
#endif


#define PZEM_DEFAULT_ADDR    0x01


class PZEM017v1
{
public:
#if defined(PZEM004_SOFTSERIAL)
    PZEM017v1(uint8_t receivePin, uint8_t transmitPin, uint8_t addr=PZEM_DEFAULT_ADDR);
#endif
    PZEM017v1(HardwareSerial* port, uint8_t addr=PZEM_DEFAULT_ADDR);
    ~PZEM017v1();


    float voltage();
    float current();
    float power();
    float energy();

    bool getParameters();
    float getHighvoltAlarmValue();
    float getLowvoltAlarmValue();
    uint16_t getHoldingAddress();
    uint16_t getShunttype();

    bool setAddress(uint8_t addr);
    uint8_t getAddress();

    bool setHighvoltAlarm(uint16_t volts);
    bool isHighvoltAlarmOn();

    bool setLowvoltAlarm(uint16_t volts);
    bool isLowvoltAlarmOn();

    bool setShuntType(uint16_t type);

    bool resetEnergy();

    void search();

private:

    Stream* _serial; // Serial interface
    bool _isSoft;    // Is serial interface software

    uint8_t _addr;   // Device address

    struct {
        float voltage;
        float current;
        float power;
        float energy;
        uint16_t HVAlarms;
        uint16_t LVAlarms;
    }  _currentValues; // Measured values

    struct {
        float HVAlarmVoltage;
        float LVAlarmVoltage;
        uint16_t address;
        uint16_t shunttype;
    }  _parameterValues; // Parameter values

    uint64_t _lastInputRead; // Last time input values were updated
    uint64_t _lastHoldingRead; // Last time input values were updated

    void init(uint8_t addr); // Init common to all constructors

    bool updateValues();    // Get most up to date values from device registers and cache them
    uint16_t recieve(uint8_t *resp, uint16_t len); // Receive len bytes into a buffer

    bool sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check=false, uint16_t slave_addr=0xFFFF); // Send 8 byte command

    void setCRC(uint8_t *buf, uint16_t len);           // Set the CRC for a buffer
    bool checkCRC(const uint8_t *buf, uint16_t len);   // Check CRC of buffer

    uint16_t CRC16(const uint8_t *data, uint16_t len); // Calculate CRC of buffer
};

#endif // PZEM017_H
