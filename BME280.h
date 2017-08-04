/*******************************************************************************************************************
** Class definition header for the Bosch BME280 temperature / humidity / pressure sensor. The sensor is described **
** at https://www.bosch-sensortec.com/bst/products/all_products/bme280 and the datasheet is available from Bosch  **
** at https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-11.pdf                      **
**                                                                                                                **
** The BME280 can use either SPI or I2C for communications. The initial library version 1.0.0 defines only I2C    **
** while subsequent versions also allow SPI communications                                                        **
**                                                                                                                **
** The most recent version of the library is available at https://github.com/SV-Zanshin/BME280 and extensive      **
** documentation of the library as well as example programs are described in the project's wiki pages located at  **
** https://github.com/SV-Zanshin/BME280/wiki.                                                                     **
**                                                                                                                **
** The BME280 is a very small package so it is unlikely for an Arduino hobbyist to play around with directly, the **
** hardware used to develop this library is a breakout board from AdaFruit which is well-documented at            **
** https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout.           **
**                                                                                                                **
** Although programming for the Arduino and in c/c++ is new to me, I'm a professional programmer and have learned,**
** over the years, that it is much easier to ignore superfluous comments than it is to decipher non-existent ones;**
** so both my comments and variable names tend to be verbose. The code is written to fit in the first 80 spaces   **
** and the comments start after that and go to column 117 - allowing the code to be printed in A4 landscape mode. **
**                                                                                                                **
** This program is free software: you can redistribute it and/or modify it under the terms of the GNU General     **
** Public License as published by the Free Software Foundation, either version 3 of the License, or (at your      **
** option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY     **
** WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the   **
** GNU General Public License for more details. You should have received a copy of the GNU General Public License **
** along with this program.  If not, see <http://www.gnu.org/licenses/>.                                          **
**                                                                                                                **
** Vers.  Date       Developer           Comments                                                                 **
** ====== ========== =================== ======================================================================== **
** 1.0.1  2017-08-03 Arnd@SV-Zanshin.Com All read/writes now use getData() and putData() templates in this header **
**                                       changed begin() method for I2C to search for first instance of BME280    **
** 1.0.0  2017-08-03 Arnd@SV-Zanshin.Com Initial version with just I2C connectivity                               **
** 1.0.0b 2017-07-31 Arnd@SV-Zanshin.Com Continued development                                                    **
** 1.0.0a 2017-07-30 Arnd@SV-Zanshin.Com Started coding                                                           **
**                                                                                                                **
*******************************************************************************************************************/
#include "Arduino.h"                                                          // Arduino data type definitions    //
#include <Wire.h>                                                             // Standard I2C "Wire" library      //
#ifndef BME280_h                                                              // Guard code definition            //
  #define BME280_h                                                            // Define the name inside guard code//
  /*****************************************************************************************************************
  ** Declare constants used in the class                                                                          **
  *****************************************************************************************************************/
  const uint8_t  BME280_CHIPID_REG       = 0xD0;                              // Chip-Id register                 //
  const uint8_t  BME280_CHIPID           = 0x60;                              // Hard-coded value 0x60 for BME280 //
  const uint8_t  BME280_SOFTRESET_REG    = 0xE0;                              // Reset when 0xB6 is written here  //
  const uint8_t  BME280_CONTROLHUMID_REG = 0xF2;                              // Humidity control register        //
  const uint8_t  BME280_STATUS_REG       = 0xF3;                              // Device status register           //
  const uint8_t  BME280_CONTROL_REG      = 0xF4;                              // Device control register          //
  const uint8_t  BME280_CONFIG_REG       = 0xF5;                              // Device configuration register    //
  const uint8_t  BME280_PRESSUREDATA_REG = 0xF7;                              // Pressure readings register       //
  const uint8_t  BME280_TEMPDATA_REG     = 0xFA;                              // Temperature readings register    //
  const uint8_t  BME280_HUMIDDATA_REG    = 0xFD;                              // Humidity readings register       //
  const uint8_t  BME280_SOFTWARE_CODE    = 0xB6;                              // Reset on this written to resetreg//
                                                                              //----------------------------------//
  const uint8_t  BME280_T1_REG           = 0x88;                              // Declare BME280 registers for the //
  const uint8_t  BME280_T2_REG           = 0x8A;                              // calibration data used to convert //
  const uint8_t  BME280_T3_REG           = 0x8C;                              // the raw measurements into metric //
  const uint8_t  BME280_P1_REG           = 0x8E;                              // units                            //
  const uint8_t  BME280_P2_REG           = 0x90;                              //                                  //
  const uint8_t  BME280_P3_REG           = 0x92;                              //                                  //
  const uint8_t  BME280_P4_REG           = 0x94;                              //                                  //
  const uint8_t  BME280_P5_REG           = 0x96;                              //                                  //
  const uint8_t  BME280_P6_REG           = 0x98;                              //                                  //
  const uint8_t  BME280_P7_REG           = 0x9A;                              //                                  //
  const uint8_t  BME280_P8_REG           = 0x9C;                              //                                  //
  const uint8_t  BME280_P9_REG           = 0x9E;                              //                                  //
  const uint8_t  BME280_H1_REG           = 0xA1;                              //                                  //
  const uint8_t  BME280_H2_REG           = 0xE1;                              //                                  //
  const uint8_t  BME280_H3_REG           = 0xE3;                              //                                  //
  const uint8_t  BME280_H4_REG           = 0xE4;                              //                                  //
  const uint8_t  BME280_H5_REG           = 0xE5;                              //                                  //
  const uint8_t  BME280_H6_REG           = 0xE7;                              //----------------------------------//
  /*****************************************************************************************************************
  ** Declare enumerated types used in the class                                                                   **
  *****************************************************************************************************************/
  enum modeTypes         {SleepMode,ForcedMode,ForcedMode2,NormalMode,        // Enumerate device modes           //
                          UnknownMode };                                      //                                  //
  enum sensorTypes       {TemperatureSensor,HumiditySensor,PressureSensor,    // Enumerate the sensor types       //
                          UnknownSensor};                                     //                                  //
  enum oversamplingTypes {SensorOff,Oversample1,Oversample2,Oversample4,      // Enumerate oversampling values    //
                          Oversample8,Oversample16,UnknownOversample };       //                                  //
  enum iirFilterTypes    {IIROff, IIR2, IIR4, IIR8, IIR16, UnknownIIR };      // Enumerate IIR filter values      //
  enum inactiveTimeTypes {inactiveHalf,inactive63ms,inactive125ms,            //                                  //
                          inactive250ms,inactive500ms,inactive1000ms,         //                                  //
                          inactive10ms,inactive20ms,UnknownInactive };        //                                  //
  enum measureTimeTypes  {TypicalMeasure,MaximumMeasure,UnknownMeasure };     //                                  //
  /*****************************************************************************************************************
  ** Main BME280 class for the temperature / humidity / pressure sensor                                           **
  *****************************************************************************************************************/
  class BME280_Class {                                                        // Class definition                 //
    public:                                                                   // Publicly visible methods         //
      BME280_Class();                                                         // Class constructor                //
      ~BME280_Class();                                                        // Class destructor                 //
      bool     begin();                                                       // Start using I2C Communications   //
      uint8_t  mode();                                                        // return device mode               //
      uint8_t  mode(const uint8_t operatingMode);                             // Set device mode                  //
      bool     setOversampling(const uint8_t sensor, const uint8_t sampling); // Set enum sensorType Oversampling //
      uint8_t  getOversampling(const uint8_t sensor,                          // Get enum sensorType oversampling //
                               const bool    actual = false);                 // if "actual" set then return #    //
      uint8_t  iirFilter();                                                   // Return the IIR Filter setting    //
      uint8_t  iirFilter(const uint8_t iirFilterSetting );                    // Set IIR Filter and return value  //
      uint8_t  inactiveTime(const uint8_t inactiveTimeSetting=UINT8_MAX);     // Set inactive time & return value //
      uint32_t measurementTime(const uint8_t measureTimeSetting=1);           // Return measurement cycle time    //
      void     getSensorData(int32_t &temp, int32_t &hum, int32_t &press);    // get most recent readings         //
      void     reset();                                                       // Reset the BME280                 //
    private:                                                                  // -------- Private methods ------- //
      uint8_t  readByte(const uint8_t addr);                                  // Read byte from register address  //
      void     readSensors();                                                 // read the registers in one burst  //
      void     getCalibration();                                              // Load calibration from registers  //
      bool     _TransmissionStatus = false;                                   // I2C communications status        //
      uint8_t  _I2CAddress         = 0;                                       // Default is no I2C address known  //
      uint8_t  _cal_dig_H1,_cal_dig_H3;                                       // Declare all of the calibration   //
      int8_t   _cal_dig_H6         = 0;                                       // variables                        //
      uint16_t _cal_dig_T1,_cal_dig_P1;                                       //                                  //
      int16_t  _cal_dig_T2,_cal_dig_T3,_cal_dig_P2,_cal_dig_P3,_cal_dig_P4,   //                                  //
               _cal_dig_P5,_cal_dig_P6,_cal_dig_P7,_cal_dig_P8,_cal_dig_P9,   //                                  //
               _cal_dig_H2,_cal_dig_H4,_cal_dig_H5;                           //                                  //
      uint8_t  _mode = UINT8_MAX;                                             // Last mode set                    //
      int32_t  _tfine;                                                        // Global calibration value         //
      int32_t  _Temperature,_Pressure,_Humidity;                              // Store the last readings          //
      /*************************************************************************************************************
      ** Declare the getData and putData methods as template functions. All device I/O is done through these two  **
      ** functions regardless of whether I2C, hardware SPI or software SPI is being used. The two functions are   **
      ** designed so thatonly the address and a variable are passed in and the functions determine the size of    **
      ** the parameter variable and reads or writes that many bytes. So if a read is called using a character     **
      ** array[10] then 10 bytes are read, if called with a int8 then only one byte is read.                      **
      ** This is done by using template function definitions which need to be defined in this header file rather  **
      ** than in the c++ program library file.                                                                    **
      *************************************************************************************************************/
      template< typename T > uint8_t &getData(const uint8_t addr,T &value) {  // method to write a structure      //
        uint8_t* bytePtr    = (uint8_t*)&value;                               // Pointer to structure beginning   //
        uint8_t  structSize = sizeof(T);                                      // Number of bytes in structure     //
        Wire.beginTransmission(_I2CAddress);                                  // Address the I2C device           //
        Wire.write(addr);                                                     // Send register address to read    //
        _TransmissionStatus = Wire.endTransmission();                         // Close transmission               //
        Wire.requestFrom(_I2CAddress, sizeof(T));                             // Request 1 byte of data           //
        while(!Wire.available());                                             // Wait until the first byte ready  //
        for (uint16_t i=0;i<structSize;i++) *bytePtr++ = Wire.read();         // loop for each byte to be read    //
        return(structSize);                                                   // return the number of bytes read  //
      } // of method getData()                                                //----------------------------------//
      template<typename T>uint8_t &putData(const uint8_t addr,const T &value){// method to write a structure      //
        const uint8_t* bytePtr = (const uint8_t*)&value;                      // Pointer to structure beginning   //
        uint8_t  structSize   = sizeof(T);                                    // Number of bytes in structure     //
        Wire.beginTransmission(_I2CAddress);                                  // Address the I2C device           //
        Wire.write(addr);                                                     // Send register address to write   //
        for (uint8_t i=0;i<sizeof(T);i++) Wire.write(*bytePtr++);             // loop for each byte to be written //
        _TransmissionStatus = Wire.endTransmission();                         // Close transmission               //
        return(structSize);                                                   // return number of bytes written   //
      } // of method putData()                                                //----------------------------------//
  }; // of BME280 class definition                                            //                                  //
#endif                                                                        //----------------------------------//
