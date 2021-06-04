#include "mbed.h"
#include "scd30.h"

//-----------------------------------------------------------------------------
// Constructor 

SCD30::SCD30(PinName sda, PinName scl, int i2cFrequency)  : _i2c(sda, scl) {
        _i2c.frequency(i2cFrequency);
}

//-----------------------------------------------------------------------------
// Destructor

SCD30::~SCD30() {
}

//-----------------------------------------------------------------------------
// start auto-measurement with barometer reading (in mB)
//

uint8_t SCD30::startMeasurement(uint16_t baro)
{
    i2cBuff[0] = SCD30_CMMD_STRT_CONT_MEAS >> 8;
    i2cBuff[1] = SCD30_CMMD_STRT_CONT_MEAS & 255;
    i2cBuff[2] = baro >> 8;
    i2cBuff[3] = baro & 255;
    i2cBuff[4] = SCD30::calcCrc2b(baro);
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 5, false);
    if(res) return SCDnoAckERROR;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Stop auto-measurement

uint8_t SCD30::stopMeasurement()
{
    i2cBuff[0] = SCD30_CMMD_STOP_CONT_MEAS >> 8;
    i2cBuff[1] = SCD30_CMMD_STOP_CONT_MEAS & 255;
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 2, false);
    if(res) return SCDnoAckERROR;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Change the measurement interval (in Secs), 

uint8_t SCD30::setMeasInterval(uint16_t mi)
{
    i2cBuff[0] = SCD30_CMMD_SET_MEAS_INTVL >> 8;
    i2cBuff[1] = SCD30_CMMD_SET_MEAS_INTVL & 255;
    i2cBuff[2] = mi >> 8;
    i2cBuff[3] = mi & 255;
    i2cBuff[4] = SCD30::calcCrc2b(mi);
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 5, false);
    if(res) return SCDnoAckERROR;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Get ready status value

uint8_t SCD30::getReadyStatus()
{
    i2cBuff[0] = SCD30_CMMD_GET_READY_STAT >> 8;
    i2cBuff[1] = SCD30_CMMD_GET_READY_STAT & 255;
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 2, false);
    if(res) return SCDnoAckERROR;
    
    _i2c.read(SCD30_I2C_ADDR | 1, i2cBuff, 3, false);
    uint16_t stat = (i2cBuff[0] << 8) | i2cBuff[1];
    scdSTR.ready = stat;
    uint8_t dat = SCD30::checkCrc2b(stat, i2cBuff[2]);
    
    if(dat == SCDcrcERROR) return SCDcrcERRORv1;
    if(dat == SCDisReady) return SCDisReady;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Get all the measurement values, stick them into the array

uint8_t SCD30::readMeasurement()
{
    i2cBuff[0] = SCD30_CMMD_READ_MEAS >> 8;
    i2cBuff[1] = SCD30_CMMD_READ_MEAS & 255;
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 2, false);
    if(res) return SCDnoAckERROR;
    
    _i2c.read(SCD30_I2C_ADDR | 1, i2cBuff, 18, false);
    
    uint16_t stat = (i2cBuff[0] << 8) | i2cBuff[1];
    scdSTR.co2m = stat;
    uint8_t dat = SCD30::checkCrc2b(stat, i2cBuff[2]);
    if(dat == SCDcrcERROR) return SCDcrcERRORv1;
    
    stat = (i2cBuff[3] << 8) | i2cBuff[4];
    scdSTR.co2l = stat;
    dat = SCD30::checkCrc2b(stat, i2cBuff[5]);
    if(dat == SCDcrcERROR) return SCDcrcERRORv2;
    
    stat = (i2cBuff[6] << 8) | i2cBuff[7];
    scdSTR.tempm = stat;
    dat = SCD30::checkCrc2b(stat, i2cBuff[8]);
    if(dat == SCDcrcERROR) return SCDcrcERRORv3;
    
    stat = (i2cBuff[9] << 8) | i2cBuff[10];
    scdSTR.templ = stat;
    dat = SCD30::checkCrc2b(stat, i2cBuff[11]);
    if(dat == SCDcrcERROR) return SCDcrcERRORv4;
    
    stat = (i2cBuff[12] << 8) | i2cBuff[13];
    scdSTR.humm = stat;
    dat = SCD30::checkCrc2b(stat, i2cBuff[14]);
    if(dat == SCDcrcERROR) return SCDcrcERRORv5;
    
    stat = (i2cBuff[15] << 8) | i2cBuff[16];
    scdSTR.huml = stat;
    dat = SCD30::checkCrc2b(stat, i2cBuff[17]);
    if(dat == SCDcrcERROR) return SCDcrcERRORv6;
    
    scdSTR.co2i = (scdSTR.co2m << 16) | scdSTR.co2l ;
    scdSTR.tempi = (scdSTR.tempm << 16) | scdSTR.templ ;
    scdSTR.humi = (scdSTR.humm << 16) | scdSTR.huml ;
    
    scdSTR.co2f = *(float*)&scdSTR.co2i;
    scdSTR.tempf = *(float*)&scdSTR.tempi;
    scdSTR.humf = *(float*)&scdSTR.humi;
    
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Change the ambient temperature (in 0.01C), 
//   i.e. 0x1F4 = 500 = 5.00C, CRC = 0x33

uint8_t SCD30::setTemperatureOffs(uint16_t temp)
{
    i2cBuff[0] = SCD30_CMMD_SET_TEMP_OFFS >> 8;
    i2cBuff[1] = SCD30_CMMD_SET_TEMP_OFFS & 255;
    i2cBuff[2] = temp >> 8;
    i2cBuff[3] = temp & 255;
    i2cBuff[4] = SCD30::calcCrc2b(temp);
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 5, false);
    if(res) return SCDnoAckERROR;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Change the altitude reading (in meters above seal level)

uint8_t SCD30::setAltitudeComp(uint16_t alt)
{
    i2cBuff[0] = SCD30_CMMD_SET_ALT_COMP >> 8;
    i2cBuff[1] = SCD30_CMMD_SET_ALT_COMP & 255;
    i2cBuff[2] = alt >> 8;
    i2cBuff[3] = alt & 255;
    i2cBuff[4] = SCD30::calcCrc2b(alt);
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 5, false);
    if(res) return SCDnoAckERROR;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Perform a soft reset on the SCD30

uint8_t SCD30::softReset()
{
    i2cBuff[0] = SCD30_CMMD_SOFT_RESET >> 8;
    i2cBuff[1] = SCD30_CMMD_SOFT_RESET & 255;
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 2, false);
    if(res) return SCDnoAckERROR;
    return SCDnoERROR;
}
    
//-----------------------------------------------------------------------------
// Calculate the CRC of a 2 byte value using the SCD30 CRC polynomial

uint8_t SCD30::calcCrc2b(uint16_t seed)
{
  uint8_t bit;                  // bit mask
  uint8_t crc = SCD30_CRC_INIT; // calculated checksum
  
  // calculates 8-Bit checksum with given polynomial

    crc ^= (seed >> 8) & 255;
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ SCD30_POLYNOMIAL;
      else           crc = (crc << 1);
    }

    crc ^= seed & 255;
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ SCD30_POLYNOMIAL;
      else           crc = (crc << 1);
    }
    
  return crc;
}

//-----------------------------------------------------------------------------
// Compare the CRC values

uint8_t SCD30::checkCrc2b(uint16_t seed, uint8_t crcIn)
{
    uint8_t crcCalc = SCD30::calcCrc2b(seed);
    if(crcCalc != crcIn) return SCDcrcERROR;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// start single-measurement with barometer reading (in mB)
//

uint8_t SCD30::startOneMeasurement(uint16_t baro)
{
    i2cBuff[0] = SCD30_CMMD_START_SINGLE_MEAS >> 8;
    i2cBuff[1] = SCD30_CMMD_START_SINGLE_MEAS & 255;
    i2cBuff[2] = baro >> 8;
    i2cBuff[3] = baro & 255;
    i2cBuff[4] = SCD30::calcCrc2b(baro);
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 5, false);
    if(res) return SCDnoAckERROR;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Get article code

uint8_t SCD30::getArticleCode()
{
    i2cBuff[0] = SCD30_CMMD_READ_ARTICLECODE >> 8;
    i2cBuff[1] = SCD30_CMMD_READ_ARTICLECODE & 255;
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 2, false);
    if(res) return SCDnoAckERROR;
    
    _i2c.read(SCD30_I2C_ADDR | 1, i2cBuff, 3, false);
    uint16_t stat = (i2cBuff[0] << 8) | i2cBuff[1];
    scdSTR.acode = stat;
    uint8_t dat = SCD30::checkCrc2b(stat, i2cBuff[2]);
    
    if(dat == SCDcrcERROR) return SCDcrcERRORv1;
    return SCDnoERROR;
}

//-----------------------------------------------------------------------------
// Get SCD30 serial number

uint8_t SCD30::getSerialNumber()
{
    i2cBuff[0] = SCD30_CMMD_READ_SERIALNBR >> 8;
    i2cBuff[1] = SCD30_CMMD_READ_SERIALNBR & 255;
    int res = _i2c.write(SCD30_I2C_ADDR, i2cBuff, 2, false);
    if(res) return SCDnoAckERROR;
    
    for(uint i = 0; i < sizeof(scdSTR.sn); i++) scdSTR.sn[i] = 0;
    for(uint i = 0; i < sizeof(i2cBuff); i++) i2cBuff[i] = 0;
    
    _i2c.read(SCD30_I2C_ADDR | 1, i2cBuff, SCD30_SN_SIZE, false);
    int t = 0;
    for(int i = 0; i < SCD30_SN_SIZE; i +=3) {
        uint16_t stat = (i2cBuff[i] << 8) | i2cBuff[i + 1];
        scdSTR.sn[i - t] = stat >> 8;
        scdSTR.sn[i - t + 1] = stat & 255;
        uint8_t dat = SCD30::checkCrc2b(stat, i2cBuff[i + 2]);
        t++;
        if(dat == SCDcrcERROR) return SCDcrcERRORv1;
        if(stat == 0) break;
    }

    return SCDnoERROR;
}
