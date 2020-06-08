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


#include "PZEM017v1.h"
#include <stdio.h>

#define REG_VOLTAGE     0x0000
#define REG_CURRENT     0x0001
#define REG_POWER_L     0x0002
#define REG_POWER_H     0x0003
#define REG_ENERGY_L    0x0004
#define REG_ENERGY_H    0x0005
#define REG_HVALARM     0x0006
#define REG_LVALARM     0x0007

#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42

#define SHUNT_100A      0x0000
#define SHUNT_50A       0x0001
#define SHUNT_200A      0x0002
#define SHUNT_300A      0x0003

#define WREG_HV_ALARM_THR   0x0000
#define WREG_LV_ALARM_THR   0x0001
#define WREG_ADDR           0x0002
#define WREG_SHUNT          0x0003

#define UPDATE_TIME     200

#define RESPONSE_SIZE 32
#define READ_TIMEOUT 100

#define PZEM_BAUD_RATE 9600

extern HardwareSerial Serial;

#define DEBUG

// Debugging function;
void printBuf(uint8_t* buffer, uint16_t len){
#ifdef DEBUG
    for(uint16_t i = 0; i < len; i++){
        char temp[6];
        sprintf(temp, "%.2x ", buffer[i]);
        Serial.print(temp);

    }
    Serial.println();
#endif
}

/*!
 * PZEM017v1::PZEM017v1
 *
 * Software Serial constructor
 *
 * @param receivePin RX pin
 * @param transmitPin TX pin
 * @param addr Slave address of device
*/
#if defined(PZEM004_SOFTSERIAL)
PZEM017v1::PZEM017v1(uint8_t receivePin, uint8_t transmitPin, uint8_t addr)
{
    SoftwareSerial *port = new SoftwareSerial(receivePin, transmitPin);
    port->begin(PZEM_BAUD_RATE,SWSERIAL_8N2);
    this->_serial = port;
    this->_isSoft = true;
    init(addr);
}
#endif

/*!
 * PZEM017v1::PZEM017v1
 *
 * Hardware serial constructor
 *
 * @param port Hardware serial to use
 * @param addr Slave address of device
*/
PZEM017v1::PZEM017v1(HardwareSerial* port, uint8_t addr)
{
    port->begin(PZEM_BAUD_RATE);
    this->_serial = port;
    this->_isSoft = false;
    init(addr);
}

/*!
 * PZEM017v1::~PZEM017v1
 *
 * Destructor deleting software serial
 *
*/
PZEM017v1::~PZEM017v1()
{
    if(_isSoft)
        delete this->_serial;
}

/*!
 * PZEM017v1::voltage
 *
 * Get line voltage in Volts
 *
 * @return current L-N volage
*/
float PZEM017v1::voltage()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.voltage;
}

/*!
 * PZEM017v1::current
 *
 * Get line in Amps
 *
 * @return line current
*/
float PZEM017v1::current()
{
    if(!updateValues())// Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.current;
}

/*!
 * PZEM017v1::power
 *
 * Get Active power in W
 *
 * @return active power in W
*/
float PZEM017v1::power()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.power;
}

/*!
 * PZEM017v1::energy
 *
 * Get Active energy in kWh since last reset
 *
 * @return active energy in kWh
*/
float PZEM017v1::energy()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.energy;
}

/*!
 * PZEM017v1::sendCmd8
 *
 * Prepares the 8 byte command buffer and sends
 *
 * @param[in] cmd - Command to send (position 1)
 * @param[in] rAddr - Register address (postion 2-3)
 * @param[in] val - Register value to write (positon 4-5)
 * @param[in] check - perform a simple read check after write
 *
 * @return success
*/
bool PZEM017v1::sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr){
    uint8_t sendBuffer[8]; // Send buffer
    uint8_t respBuffer[8]; // Response buffer (only used when check is true)

    if((slave_addr == 0xFFFF) ||
       (slave_addr < 0x01) ||
       (slave_addr > 0xF7)){
        slave_addr = _addr;
    }

    sendBuffer[0] = slave_addr;              // Set slave address
    sendBuffer[1] = cmd;                     // Set command

    sendBuffer[2] = (rAddr >> 8) & 0xFF;     // Set high byte of register address
    sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=

    sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
    sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=

    setCRC(sendBuffer, 8);                   // Set CRC of frame

    _serial->write(sendBuffer, 8); // send frame

    if(check) {
        if(!recieve(respBuffer, 8)){ // if check enabled, read the response
            return false;
        }

        // Check if response is same as send
        for(uint8_t i = 0; i < 8; i++){
            if(sendBuffer[i] != respBuffer[i])
                return false;
        }
    }
    return true;
}


/*!
 * PZEM017v1::setAddress
 *
 * Set a new device address and update the device
 * WARNING - should be used to set up devices once.
 * Code initializtion will still have old address on next run!
 *
 * @param[in] addr New device address 0x01-0xF7
 *
 * @return success
*/
bool PZEM017v1::setAddress(uint8_t addr)
{
    if(addr < 0x01 || addr > 0xF7) // sanity check
        return false;

    // Write the new address to the address register
    if(!sendCmd8(CMD_WSR, WREG_ADDR, addr, true))
        return false;

    _addr = addr; // If successful, update the current slave address

    return true;
}

/*!
 * PZEM017v1::getAddress
 *
 * Get the current device address
 *
 * @return address
*/
uint8_t PZEM017v1::getAddress()
{
    return _addr;
}

/*!
 * PZEM017v1::isHighVoltAlarmOn
 *
 * Is the HV alarm set
 *
 *
 * @return alarm triggerd
*/
bool PZEM017v1::isHighvoltAlarmOn()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.HVAlarms != 0x0000;
}

/*!
 * PZEM017v1::isLowVoltAlarmOn
 *
 * Is the LV alarm set
 *
 *
 * @return alarm triggerd
*/
bool PZEM017v1::isLowvoltAlarmOn()
{
    if(!updateValues()) // Update vales if necessary
        return NAN; // Update did not work, return NAN

    return _currentValues.LVAlarms != 0x0000;
}

/*!
 * PZEM017v1::init
 *
 * initialization common to all consturctors
 *
 * @param[in] addr - device address
 *
 * @return success
*/
void PZEM017v1::init(uint8_t addr){
    if(addr < 0x01 || addr > 0xF8) // Sanity check of address
        addr = PZEM_DEFAULT_ADDR;
    _addr = addr;

    // Set initial lastRed time so that we read right away
    _lastInputRead = 0;
    _lastInputRead -= UPDATE_TIME;

    _lastHoldingRead = 0;
    _lastHoldingRead -= UPDATE_TIME;
}

/*!
 * PZEM017v1::getParameters()
 *
 * Read all parameters of device and update the local values
 *
 * @return success
*/
bool PZEM017v1::getParameters()
{
    static uint8_t response[13];

    // If we read before the update time limit, do not update
    if(_lastHoldingRead + UPDATE_TIME > millis()){
        return true;
    }

    // Read 3 registers starting at 0x00
    sendCmd8(CMD_RHR, 0x00, 0x04, false);

    if(recieve(response, 13) != 13){ // Something went wrong
        return false;
    }

    // Update the current paramaters
    _parameterValues.HVAlarmVoltage = ((uint32_t)response[3] << 8 | // Raw voltage in 0.01V
                                        (uint32_t)response[4])/100.0;

    _parameterValues.LVAlarmVoltage = ((uint32_t)response[5] << 8 | // Raw voltage in 0.01V
                                        (uint32_t)response[6])/100.0;

    _parameterValues.address =        ((uint32_t)response[7] << 8 | // Raw address 0x00-0xf7
                                        (uint32_t)response[8]);

    _parameterValues.shunttype =      ((uint32_t)response[9] << 8 | // Shunt type 0x0000 - 0x0003 (100A/50A/200A/300A)
                                        (uint32_t)response[10]);
    // Record current time as _lastHoldingRead
    _lastHoldingRead = millis();

    return true;
}

/*!
 * PZEM017v1::GetHighVoltAlarmValue
 *
 * Current HV alarm value
 *
 *
 * @return alarm value
*/
float PZEM017v1::getHighvoltAlarmValue()
{
    if(!getParameters())
        return NAN;

     return _parameterValues.HVAlarmVoltage;
}

/*!
 * PZEM017v1::GetLowvoltAlarmValue
 *
 * Current LV alarm value
 *
 *
 * @return alarm value
*/
float PZEM017v1::getLowvoltAlarmValue()
{
    if(!getParameters())
        return NAN;

    return _parameterValues.LVAlarmVoltage;
}

/*!
 * PZEM017v1::address
 *
 * Current address
 *
 *
 * @return device address
*/
uint16_t PZEM017v1::getHoldingAddress()
{
    if(!getParameters())
        return NAN;

    return _parameterValues.address;
}

/*!
 * PZEM017v1::shunttype
 *
 * Current shuttype
 *
 *
 * @return device shuttype
*/
uint16_t PZEM017v1::getShunttype()
{
    if(!getParameters())
        return NAN;

    return _parameterValues.shunttype;
}

/*!
 * PZEM017v1::updateValues
 *
 * Read all registers of device and update the local values
 *
 * @return success
*/
bool PZEM017v1::updateValues()
{
    //static uint8_t buffer[] = {0x00, CMD_RIR, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00};
    static uint8_t response[21];

    // If we read before the update time limit, do not update
    if(_lastInputRead + UPDATE_TIME > millis()){
        return true;
    }

    // Read 10 registers starting at 0x00 (no check)
    sendCmd8(CMD_RIR, 0x00, 0x08, false);

    if(recieve(response, 21) != 21){ // Something went wrong
        return false;
    }
    // Update the current values
    _currentValues.voltage = ((uint32_t)response[3] << 8 | // Raw voltage in 0.01V
                              (uint32_t)response[4])/100.0;

    _currentValues.current = ((uint32_t)response[5] << 8 | // Raw voltage in 0.01A
                              (uint32_t)response[6])/100.0;

    _currentValues.power =   ((uint32_t)response[7] << 8 | // Raw power in 0.1W
                              (uint32_t)response[8] |
                              (uint32_t)response[9] << 24 |
                              (uint32_t)response[10] << 16) / 10.0;

    _currentValues.energy =  ((uint32_t)response[11] << 8 | // Raw Energy in 1Wh
                              (uint32_t)response[12] |
                              (uint32_t)response[13] << 24 |
                              (uint32_t)response[14] << 16) / 1000.0;

    _currentValues.HVAlarms =  ((uint32_t)response[15] << 8 | // Raw alarm value
                               (uint32_t)response[16]);

    _currentValues.LVAlarms =  ((uint32_t)response[17] << 8 | // Raw alarm value
                               (uint32_t)response[18]);
    // Record current time as _lastInputRead
    _lastInputRead = millis();

    //for(int i = 0; i < sizeof(response); i++){
    //    Serial.println(response[i]);
    //}

    return true;
}


/*!
 * PZEM017v1::resetEnergy
 *
 * Reset the Energy counter on the device
 *
 * @return success
*/
bool PZEM017v1::resetEnergy(){
    uint8_t buffer[] = {0x00, CMD_REST, 0x00, 0x00};
    uint8_t reply[5];
    buffer[0] = _addr;

    setCRC(buffer, 4);
    _serial->write(buffer, 4);

    uint16_t length = recieve(reply, 5);

    if(length == 0 || length == 5){
        return false;
    }

    return true;
}

/*!
 * PZEM017v1::setHighVoltAlarm
 *
 * Set HV alarm threshold in volts
 *
 * @param[in] volt Alarm theshold
 *
 * @return success
*/
bool PZEM017v1::setShuntType(uint16_t type)
{
    if (type < 0){ // Sanity check
        type = 0;
    }

    if (type > 3){
        type = 3;
    }

    // Write shunt type to the holding register
    if(!sendCmd8(CMD_WSR, WREG_SHUNT, type, true))
        return false;

    return true;
}

/*!
 * PZEM017v1::setHighVoltAlarm
 *
 * Set HV alarm threshold in volts
 *
 * @param[in] volt Alarm theshold
 *
 * @return success
*/
bool PZEM017v1::setHighvoltAlarm(uint16_t volts)
{
    if (volts < 500){ // Sanity check
        volts = 500;
    }

    if (volts > 34999){ // Sanity check
        volts = 34999;
    }

    // Write the volts threshold to the alarm register
    if(!sendCmd8(CMD_WSR, WREG_HV_ALARM_THR, volts, true))
        return false;

    return true;
}

/*!
 * PZEM017v1::setLowVoltAlarm
 *
 * Set LV alarm threshold in volts
 *
 * @param[in] volt Alarm theshold
 *
 * @return success
*/
bool PZEM017v1::setLowvoltAlarm(uint16_t volts)
{
    if (volts < 100){ // Sanity check
        volts = 100;
    }

    if (volts > 34999){ // Sanity check
        volts = 34999;
    }

    // Write the volts threshold to the alarm register
    if(!sendCmd8(CMD_WSR, WREG_LV_ALARM_THR, volts, true))
        return false;

    return true;
}

/*!
 * PZEM017v1::recieve
 *
 * Receive data from serial with buffer limit and timeout
 *
 * @param[out] resp Memory buffer to hold response. Must be at least `len` long
 * @param[in] len Max number of bytes to read
 *
 * @return number of bytes read
*/
uint16_t PZEM017v1::recieve(uint8_t *resp, uint16_t len)
{
    #ifdef PZEM004_SOFTSERIAL
        if(_isSoft)
            ((SoftwareSerial *)_serial)->listen(); // Start software serial listen
    #endif

    unsigned long startTime = millis(); // Start time for Timeout
    uint8_t index = 0; // Bytes we have read
    while((index < len) && (millis() - startTime < READ_TIMEOUT))
    {
        if(_serial->available() > 0)
        {
            uint8_t c = (uint8_t)_serial->read();

            resp[index++] = c;
        }
        yield();	// do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
    }

    // Check CRC with the number of bytes read
    if(!checkCRC(resp, index)){
        return 0;
    }

    return index;
}

/*!
 * PZEM017v1::checkCRC
 *
 * Performs CRC check of the buffer up to len-2 and compares check sum to last two bytes
 *
 * @param[in] data Memory buffer containing the frame to check
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
 * @return is the buffer check sum valid
*/
bool PZEM017v1::checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}


/*!
 * PZEM017v1::setCRC
 *
 * Set last two bytes of buffer to CRC16 of the buffer up to byte len-2
 * Buffer must be able to hold at least 3 bytes
 *
 * @param[out] data Memory buffer containing the frame to checksum and write CRC to
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
*/
void PZEM017v1::setCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}


// Pre computed CRC table
static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};


/*!
 * PZEM017v1::CRC16
 *
 * Calculate the CRC16-Modbus for a buffer
 * Based on https://www.modbustools.com/modbus_crc16.html
 *
 * @param[in] data Memory buffer containing the data to checksum
 * @param[in] len  Length of the respBuffer
 *
 * @return Calculated CRC
*/
uint16_t PZEM017v1::CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}

/*!
 * PZEM017v1::search
 *
 * Search for available devices. This should be used only for debugging!
 * Prints any found device addresses on the bus.
 * Can be disabled by defining PZEM017_DISABLE_SEARCH
*/
void PZEM017v1::search(){
#if ( not defined(PZEM017_DISABLE_SEARCH))
    static uint8_t response[7];
    for(uint16_t addr = 0x01; addr <= 0xF8; addr++){
        //Serial.println(addr);
        sendCmd8(CMD_RIR, 0x00, 0x01, false, addr);

        if(recieve(response, 7) != 7){ // Something went wrong
            continue;
        } else {

            Serial.print("Device on addr: ");
            Serial.print(addr);
        }
    }
#endif
}
