#ifndef SCD30_H
#define SCD30_H

/**
 * Driver for Sensirion SCD30, CO2 sensor module using I2C. 
 * The device also senses Temperature and Humidity.
 * 
 * @author: Kevin Braun (https://os.mbed.com/users/loopsva/code/scd30/)
 * 
 */ 

#define SCD30_I2C_ADDR                  0xc2

#define SCD30_CMMD_STRT_CONT_MEAS       0x0010
#define SCD30_CMMD_STOP_CONT_MEAS       0x0104
#define SCD30_CMMD_SET_MEAS_INTVL       0x4600
#define SCD30_CMMD_GET_READY_STAT       0x0202
#define SCD30_CMMD_READ_MEAS            0x0300
#define SCD30_CMMD_D_A_SELF_CALIB       0x5306
#define SCD30_CMMD_FORCE_CALIB_VAL      0x5204
#define SCD30_CMMD_SET_TEMP_OFFS        0x5403
#define SCD30_CMMD_SET_ALT_COMP         0x5102
#define SCD30_CMMD_SOFT_RESET           0xd304

#define SCD30_CMMD_READ_SERIALNBR       0xD033
#define SCD30_CMMD_READ_ARTICLECODE     0xD025
#define SCD30_CMMD_START_SINGLE_MEAS    0x0006
    
#define SCD30_POLYNOMIAL                0x131   // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
#define SCD30_CRC_INIT                  0xff

#define SCD30_SN_SIZE                   33      //size of the s/n ascii string + CRC values

    /** Create SCD30 controller class
     *
     * @param SCD30 class
     *
     */
class SCD30 {

public:
    enum SCDerror {
        SCDnoERROR,         //all ok
        SCDisReady,         //ready ststus register
        SCDnoAckERROR,      //no I2C ACK error
        SCDtimeoutERROR,    //I2C timeout error
        SCDcrcERROR,        //CRC error, any
        SCDcrcERRORv1,      //CRC error on value 1
        SCDcrcERRORv2,      //CRC error on value 2
        SCDcrcERRORv3,      //CRC error on value 3
        SCDcrcERRORv4,      //CRC error on value 4
        SCDcrcERRORv5,      //CRC error on value 5
        SCDcrcERRORv6,      //CRC error on value 6
    };
    
    /**
     * Structure to access SCD30's raw and finished data
     *
     */    
    struct scdSTRuct {
        uint16_t co2m;          /**< High order 16 bit word of CO2 */
        uint16_t co2l;          /**< Low  order 16 bit word of CO2 */
        uint16_t tempm;         /**< High order 16 bit word of Temp */
        uint16_t templ;         /**< Low order 16 bit word of Temp */
        uint16_t humm;          /**< High order 16 bit word of Hum */
        uint16_t huml;          /**< Low order 16 bit word of Hum */
        
        uint16_t ready;         /**< 1 = ready, 0 = busy */
        
        uint32_t co2i;          /**< 32 bit int of CO2 */
        uint32_t tempi;         /**< 32 bit int of Temp */
        uint32_t humi;          /**< 32 bit int of Hum */
        
        float co2f;             /**< float of CO2 concentration */
        float tempf;            /**< float of Temp */
        float humf;             /**< float of Hum */
        
        uint16_t acode;         /**< Article code number?? */
        uint8_t sn[24];         /**< ASCII Serial Number */
    } scdSTR;
    
    
    /** Create a SCD30 object using the specified I2C object
     * @param sda - mbed I2C interface pin
     * @param scl - mbed I2C interface pin
     * @param I2C Frequency (in Hz)
     *
     * @return none
     */
     SCD30(PinName sda, PinName scl, int i2cFrequency);
     
    /** Destructor
     *
     * @param --none--
     *
     * @return none
     */
    ~SCD30();
    
    /** Start Auto-Measurement 
     *
     * @param Barometer reading (in mB) or 0x0000
     *
     * @return enum SCDerror
     */
    uint8_t startMeasurement(uint16_t baro);
    
    /** Stop Auto-Measurement 
     *
     * @param --none--
     *
     * @return enum SCDerror
     */
    uint8_t stopMeasurement();
    
    /** Set Measurement Interval 
     *
     * @param Time between measurements (in seconds)
     *
     * @return enum SCDerror
     */
    uint8_t setMeasInterval(uint16_t mi);
    
    /** Get Ready Status register 
     *
     * @param --none--
     * @see Ready Status result in scdSTR structure
     *
     * @return enum SCDerror
     */
    uint8_t getReadyStatus();
    
    /** Get all environmental parameters (CO2, Temp and Hum) 
     *
     * @param --none-
     * @see Results in scdSTR structure
     *
     * @return enum SCDerror
     */
    uint8_t readMeasurement();
    
    /** Set Temperature offset
     *
     * @param Temperature offset (value in 0.01 degrees C)
     *
     * @return enum SCDerror
     */
    uint8_t setTemperatureOffs(uint16_t temp);
    
    /** Set Altitude Compensation
     *
     * @param Altitude (in meters)
     *
     * @return enum SCDerror
     */
    uint8_t setAltitudeComp(uint16_t alt);
    
    /** Perform a soft reset
     *
     * @param --none--
     *
     * @return enum SCDerror
     */
    uint8_t softReset();
    
    /** Calculate the SCD30 CRC value
     *
     * @param 16 bit value to perform a CRC check on
     *
     * @return 8 bit CRC value
     */
    uint8_t calcCrc2b(uint16_t seed);
    
    /** Compare received CRC value with calculated CRC value
     *
     * @param 16 bit value to perform a CRC check on
     * @param 8 bit value to compare CRC values
     *
     * @return enum SCDerror
     */
    uint8_t checkCrc2b(uint16_t seed, uint8_t crcIn);
    
    /** Start a Single-Measurement 
     *
     * @param Barometer reading (in mB) or 0x0000
     *
     * @return enum SCDerror
     */
    uint8_t startOneMeasurement(uint16_t baro);
    
    /** Get Article Code
     *
     * @param --none--
     * 
     *
     * @return enum SCDerror
     */
    uint8_t getArticleCode();
    
    /** Get Serial Number
     *
     * @param --none--
     * @see ASCII Serial Number in scdSTR structure
     *
     * @return enum SCDerror
     */
    uint8_t getSerialNumber();
 
private:
    char i2cBuff[34];
 
protected:
    I2C     _i2c;    

};    
#endif

