/*******************************************************************************************************************
** BME280 class method definitions. See the header file for program details and version information               **
**                                                                                                                **
** This program is free software: you can redistribute it and/or modify it under the terms of the GNU General     **
** Public License as published by the Free Software Foundation, either version 3 of the License, or (at your      **
** option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY     **
** WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the   **
** GNU General Public License for more details. You should have received a copy of the GNU General Public License **
** along with this program.  If not, see <http://www.gnu.org/licenses/>.                                          **
**                                                                                                                **
*******************************************************************************************************************/
#include "BME280.h"                                                           // Include the header definition    //
                                                                              //                                  //
/*******************************************************************************************************************
** Class Constructor instantiates the class                                                                       **
*******************************************************************************************************************/
BME280_Class::BME280_Class()  {} // of class constructor                      //                                  //
/*******************************************************************************************************************
** Class Destructor currently does nothing and is included for compatibility purposes                             **
*******************************************************************************************************************/
BME280_Class::~BME280_Class() {} // of class destructor                       //                                  //
/*******************************************************************************************************************
** Method begin starts I2C communications with the device, using a default address if one is not specified and    **
** return true if the device has been detected and false if it was not                                            **
*******************************************************************************************************************/
bool BME280_Class::begin(const uint8_t I2CAddress ) {                         // Start I2C communications         //
  Wire.begin();                                                               // Start I2C as master device       //
  _I2CAddress = I2CAddress;                                                   // Store actual I2C address to use  //
  Wire.beginTransmission(_I2CAddress);                                        // Address the BME280               //
  delay(BME280_I2C_DELAY);                                                    // Give the BME280 time to process  //
  uint8_t errorCode = Wire.endTransmission();                                 // See if there's a device present  //
  if (errorCode == 0) {                                                       // If we have a device at address,  //
    if (readByte(BME280_CHIPID_REG)==BME280_CHIPID) {                         // and it returns correct chip id,  //
      getCalibration();                                                       // get the calibration values       //
      return true;                                                            // return success                   //
    } // of if-then device is really a BME280                                 //                                  //
  } // of if-then device detected                                             //                                  //
  return false;                                                               // return failure if we get here    //
} // of method begin()                                                        //                                  //
/*******************************************************************************************************************
** Method getCalibration reads the calibration register data into local variables for use in converting readings  **
*******************************************************************************************************************/
void BME280_Class::getCalibration() {                                         // Read and store registers         //
  getData(BME280_T1_REG,_cal_dig_T1);                                         // Retrieve calibration values      //
  getData(BME280_T2_REG,_cal_dig_T2);                                         //                                  //
  getData(BME280_T3_REG,_cal_dig_T3);                                         //                                  //
  getData(BME280_P1_REG,_cal_dig_P1);                                         //                                  //
  getData(BME280_P2_REG,_cal_dig_P2);                                         //                                  //
  getData(BME280_P3_REG,_cal_dig_P3);                                         //                                  //
  getData(BME280_P4_REG,_cal_dig_P4);                                         //                                  //
  getData(BME280_P5_REG,_cal_dig_P5);                                         //                                  //
  getData(BME280_P6_REG,_cal_dig_P6);                                         //                                  //
  getData(BME280_P7_REG,_cal_dig_P7);                                         //                                  //
  getData(BME280_P8_REG,_cal_dig_P8);                                         //                                  //
  getData(BME280_P9_REG,_cal_dig_P9);                                         //                                  //
  getData(BME280_H1_REG,_cal_dig_H1);                                         //                                  //
  getData(BME280_H2_REG,_cal_dig_H2);                                         //                                  //
  getData(BME280_H3_REG,_cal_dig_P3);                                         //                                  //
  uint8_t tempVar;                                                            // Single-Byte temporary variable   //
  getData(BME280_H4_REG,tempVar);                                             // Retrieve byte                    //
  _cal_dig_H4 = tempVar<<4;                                                   //                                  //
  getData(BME280_H4_REG+1,tempVar);                                           // Retrieve byte                    //
  _cal_dig_H4 |= tempVar&0xF;                                                 //                                  //
  getData(BME280_H5_REG+2,tempVar);                                           // Retrieve byte                    //
  _cal_dig_H5 = tempVar<<4;                                                   //                                  //
  getData(BME280_H5_REG,tempVar);                                             // Retrieve byte                    //
  _cal_dig_H5 |= tempVar>>4;                                                  //                                  //
  getData(BME280_H6_REG,_cal_dig_H6);                                         //                                  //
  _cal_dig_H5 = (readByte(BME280_H5_REG+1)<<4)|                               //                                  //
                (readByte(BME280_H5_REG)>>4);                                 //                                  //
  _cal_dig_H6 = readByte(BME280_H6_REG);                                      //                                  //
} // of method getCalibration()                                               //                                  //
/*******************************************************************************************************************
** Method readByte is an interlude function to the getData() function. Reads 1 byte from the given address        **
*******************************************************************************************************************/
uint8_t BME280_Class::readByte(const uint8_t addr) {                          //                                  //
  uint8_t returnValue;                                                        // Storage for returned value       //
  getData(addr,returnValue);                                                  // Read just one byte               //
  return (returnValue);                                                       // Return byte just read            //
} // of method readByte()                                                     //                                  //
/*******************************************************************************************************************
** Method mode() returns the current mode when called with no parameters, otherwise it sets the mode and returns  **
** the mode that was set.                                                                                         **
*******************************************************************************************************************/
uint8_t BME280_Class::mode() {                                                // Return the mode                  //
  uint8_t returnMode = readByte(BME280_CONTROL_REG) & B00000011;              // Read the 2 mode bits             //
  return(returnMode);                                                         // Return the value read            //
} // of method mode()                                                         //                                  //
/*******************************************************************************************************************
** Overloaded method mode() sets the current mode bits                                                            **
*******************************************************************************************************************/
uint8_t BME280_Class::mode(const uint8_t operatingMode) {                     // Set device mode                  //
  _mode = operatingMode&B00000011;                                            // Mask 2 bits in input parameter   //
  putData(BME280_CONTROL_REG,(readByte(BME280_CONTROL_REG)&B11111100)|_mode); // Write value back to register     //
  return(_mode);                                                              // and return that value            //
} // of method mode()                                                         //                                  //
/*******************************************************************************************************************
** method setOversampling() sets the oversampling mode for the sensor (see enumerated sensorTypes) to a valid     **
** oversampling rate as defined in the enumerated type oversamplingTypes. If either value is out of range or      **
** another error occurs then the return value is false.                                                           **
*******************************************************************************************************************/
bool BME280_Class::setOversampling(const uint8_t sensor,                      // Set enum sensorType to Oversample//
                                   const uint8_t sampling) {                  //                                  //
  if(sensor>=UnknownSensor || sampling>=UnknownOversample) return(false);     // return error if out of range     //
  uint8_t tempControl = readByte(BME280_CONTROL_REG);                         // Read the control register        //
  delay(60);                                                                  // Give the BME280 a bit of time    //
  putData(BME280_CONTROL_REG,0);                                              // Put BME280 into sleep mode       //
  if(sensor==HumiditySensor) {                                                // If we have a humidity setting,   //
    putData(BME280_CONTROLHUMID_REG,sampling);                                // Update humidity register         //
    delay(60);                                                                // Give the BME280 a bit of time    //
    putData(BME280_CONTROL_REG,tempControl);                                  // Restore register values          //
    delay(60);                                                                // Give the BME280 a bit of time    //
  } else if (sensor==TemperatureSensor) {                                     // otherwise if we have temperature //
    tempControl = (tempControl&B00011111)|(sampling<<5);                      // Update the register bits         //
  } else {                                                                    //                                  //
    tempControl = (tempControl&B11100011)|(sampling<<2);                      // Update the register bits         //
  } // of if-then-else temperature reading                                    //                                  //
  putData(BME280_CONTROL_REG,tempControl);                                    // Write value to the register      //
  return(true);                                                               // return success                   //
} // of method setOversampling()                                              //                                  //
/*******************************************************************************************************************
** method getOversampling() retrieves the oversampling value (see enum oversamplingTypes) for the enumerated      **
** sensor type (see enumerated sensorTypes)                                                                       **
*******************************************************************************************************************/
uint8_t BME280_Class::getOversampling(const uint8_t sensor,                   // Get enum sensorType Oversampling //
                                      const bool    actual) {                 // if "actual" then return number   //
  uint8_t returnValue;                                                        // Get space for return value       //
  if(sensor>=UnknownSensor) return(0);                                        // return a zero if out of range    //
  if(sensor==HumiditySensor)                                                  // If we have a humidity setting,   //
    returnValue = readByte(BME280_CONTROLHUMID_REG)&B00000111;                // read the buffer bits             //
  else if (sensor==TemperatureSensor)                                         // otherwise if we have temperature //
    returnValue = readByte(BME280_CONTROL_REG)>>5;                            //                                  //
  else                                                                        //                                  //
    returnValue = (readByte(BME280_CONTROL_REG)>>2)&B00000111;                //                                  //
  if (actual) {                                                               // If the actual flag has been set, //
    if      (returnValue==3) returnValue = 4;                                 // then return the oversampling     //
    else if (returnValue==4) returnValue = 8;                                 // multiplier instead of the table  //
    else if (returnValue>4)  returnValue = 16;                                // index value                      //
  } // of if-then we return the actual count                                  //                                  //
  return(returnValue);                                                        // return oversampling bits         //
} // of method getOversampling()                                              //                                  //
/*******************************************************************************************************************
** method readSensors() reads all 3 sensor values from the registers in one operation and then proceeds to        **
** convert the raw temperature, pressure and humidity readings into standard metric units. The formula is written **
** in the BME280's documentation but the math used below was taken from Adafruit's Adafruit_BME280_Library at     **
** https://github.com/adafruit/Adafruit_BME280_Library. I think it can be refactored into more efficient code at  **
** point in the future, but it does work correctly.                                                               **
*******************************************************************************************************************/
void BME280_Class::readSensors() {                                            // read the registers in one burst  //
  uint8_t registerBuffer[8];                                                  // declare array to store registers //
  int64_t i, j, p;                                                            // Work variables                   //
  if((_mode==ForcedMode||_mode==ForcedMode2)&&mode()==SleepMode) mode(_mode); // Force a reading if necessary     //
  while(readByte(BME280_STATUS_REG)&B00001001!=0);                            // wait for measurement to complete //
  getData(BME280_PRESSUREDATA_REG,registerBuffer);                            // read all 8 bytes in one go       //
  /* First compute the temperature so that we can get the "_tfine" variable, which is used to compensate both the **
  ** humidity and pressure readings                                                                               */
  _Temperature = (int32_t)registerBuffer[3]<<12|(int32_t)registerBuffer[4]<<4|//                                  //
  (int32_t)registerBuffer[5]>>4;                                              //                                  //
  i         = ((((_Temperature>>3)-((int32_t)_cal_dig_T1 <<1)))*              //                                  //
                 ((int32_t)_cal_dig_T2))>>11;                                 //                                  //
  j         = (((((_Temperature>>4)-((int32_t)_cal_dig_T1))*                  //                                  //
                 ((_Temperature>>4)-((int32_t)_cal_dig_T1)))>>12)             //                                  //
                 *((int32_t)_cal_dig_T3))>>14;                                //                                  //
  _tfine       = i + j;                                                       //                                  //
  _Temperature = (_tfine * 5 + 128) >> 8;                                     // In centi-degrees Celsius         //
  /* Now compute the pressure value                                                                               */
  _Pressure    = (int32_t)registerBuffer[0]<<12|(int32_t)registerBuffer[1]<<4|//                                  //
                 (int32_t)registerBuffer[2]>>4;                               //                                  //
  i = ((int64_t)_tfine) - 128000;                                             //                                  //
  j = i * i * (int64_t)_cal_dig_P6;                                           //                                  //
  j = j + ((i*(int64_t)_cal_dig_P5)<<17);                                     //                                  //
  j = j + (((int64_t)_cal_dig_P4)<<35);                                       //                                  //
  i = ((i*i*(int64_t)_cal_dig_P3)>>8)+((i*(int64_t)_cal_dig_P2)<<12);         //                                  //
  i = (((((int64_t)1)<<47)+i))*((int64_t)_cal_dig_P1)>>33;                    //                                  //
  if (i == 0) _Pressure = 0;                                                  // avoid division by 0 exception    //
  else {                                                                      //                                  //
    p = 1048576 - _Pressure;                                                  //                                  //
    p = (((p<<31) - j)*3125) / i;                                             //                                  //
    i = (((int64_t)_cal_dig_P9) * (p>>13) * (p>>13)) >> 25;                   //                                  //
    j = (((int64_t)_cal_dig_P8) * p) >> 19;                                   //                                  //
    p = ((p + i + j) >> 8) + (((int64_t)_cal_dig_P7)<<4);                     //                                  //
    _Pressure = p>>8;                                                         // in pascals                       //
  } // of if pressure would cause error                                       //                                  //
  /* Now compute the Humidity value                                                                               */
  _Humidity     = (int32_t)registerBuffer[6]<<8|(int32_t)registerBuffer[7];   //                                  //
  i = (_tfine - ((int32_t)76800));                                            //                                  //
  i = (((((_Humidity<<14)-(((int32_t)_cal_dig_H4)<<20)-(((int32_t)_cal_dig_H5)//                                  //
      *i))+((int32_t)16384))>>15)*(((((((i*((int32_t)_cal_dig_H6))>>10)*      //                                  //
      (((i*((int32_t)_cal_dig_H3))>>11)+((int32_t)32768)))>>10)+              //                                  //
      ((int32_t)2097152)) * ((int32_t)_cal_dig_H2) + 8192)>>14));             //                                  //
  i = (i-(((((i>>15)*(i>>15))>>7)*((int32_t)_cal_dig_H1))>>4));               //                                  //
  i = (i < 0) ? 0 : i;                                                        //                                  //
  i = (i > 419430400) ? 419430400 : i;                                        //                                  //
  float h = (i>>12);                                                          //                                  //
  _Humidity = (uint32_t)(i>>12)*100/1024;                                     // in percent * 100                 //
} // of method readSensors()                                                  //                                  //
/*******************************************************************************************************************
** Overloaded method iirFilter() when called with no parameters returns the current IIR Filter setting, otherwise **
** when called with one parameter will set the IIR filter value and return the new setting                        **
*******************************************************************************************************************/
uint8_t BME280_Class::iirFilter() {                                           // return the IIR Filter setting    //
  uint8_t returnValue = (readByte(BME280_CONFIG_REG)>>2)&B00000111;           // Get 3 bits for the IIR Filter    //
  return(returnValue);                                                        // Return IIR Filter setting        //
} // of method iirFilter()                                                    //                                  //
uint8_t BME280_Class::iirFilter(const uint8_t iirFilterSetting ) {            // set the IIR Filter value         //
  uint8_t returnValue = readByte(BME280_CONFIG_REG)&B11110001;                // Get control reg, mask IIR bits   //
  returnValue |= (iirFilterSetting&B00000111)<<2;                             // use 3 bits of iirFilterSetting   //
  putData(BME280_CONFIG_REG,returnValue);                                     // Write new control register value //
  returnValue = (returnValue>>2)&B00000111;                                   // Extract IIR filter setting       //
  return(returnValue);                                                        // Return IIR Filter setting        //
} // of method iirFilter()                                                    //                                  //
/*******************************************************************************************************************
** Overloaded method inactiveTime() when called with no parameters returns the current inactive time setting,     **
** otherwise when called with one parameter will set the inactive time and return the new setting                 **
*******************************************************************************************************************/
uint8_t BME280_Class::inactiveTime() {                                        // return the IIR Filter setting    //
  uint8_t returnValue = (readByte(BME280_CONFIG_REG)>>5)&B00000111;           // Get 3 bits for the inactive time //
  return(returnValue);                                                        // Return IIR Filter setting        //
} // of method inactiveTime()                                                 //                                  //
uint8_t BME280_Class::inactiveTime(const uint8_t inactiveTimeSetting) {       // return the IIR Filter setting    //
  uint8_t returnValue = readByte(BME280_CONFIG_REG)&B10001111;                // Get control reg, mask inactive   //
  returnValue |= (inactiveTimeSetting&B00000111)<<5;                          // use 3 bits of inactiveTimeSetting//
  putData(BME280_CONFIG_REG,returnValue);                                     // Write new control register value //
  returnValue = (returnValue>>5)&B00000111;                                   // Extract inactive setting         //
  return(returnValue);                                                        // Return inactive time setting     //
} // of method inactiveTime()                                                 //                                  //
/*******************************************************************************************************************
** Method measurementTime() returns the time in microseconds for a measurement cycle with the current settings.   **
** A cycle includes a temperature, pressure and humidity reading plus the wait time.                              **
*******************************************************************************************************************/
uint32_t BME280_Class::measurementTime(const uint8_t measureTimeSetting) {    // return a cycle time in micros    //
  uint32_t Time1, Time2;                                                      // Declare local variables          //
  uint32_t returnValue = inactiveTime();                                      // Get inactive time value          //
  switch (returnValue) {                                                      // Set inactive time according to   //
    case inactiveHalf:   returnValue =     500;break;                         //                                  //
    case inactive63ms:   returnValue =   62500;break;                         //                                  //
    case inactive125ms:  returnValue =  125000;break;                         //                                  //
    case inactive250ms:  returnValue =  250000;break;                         //                                  //
    case inactive500ms:  returnValue =  500000;break;                         //                                  //
    case inactive1000ms: returnValue = 1000000;break;                         //                                  //
    case inactive10ms:   returnValue =   10000;break;                         //                                  //
    case inactive20ms:   returnValue =   20000;break;                         //                                  //
  } // of switch for inactive time code                                       //                                  //
  if (measureTimeSetting!=TypicalMeasure) {                                   // Set timing factors for Typ / Max //
    returnValue += 1250;                                                      //                                  //
    Time1        = 2300;                                                      //                                  //
    Time2        =  575;                                                      //                                  //
  } else {                                                                    //                                  //
    returnValue += 1000;                                                      //                                  //
    Time1        = 2000;                                                      //                                  //
    Time2        =  500;                                                      //                                  //
  } // of if-then-else typical time or maximum time                           //                                  //
  if (getOversampling(TemperatureSensor))                                     //                                  //
    returnValue += Time1*getOversampling(TemperatureSensor,true);             //                                  //
  if (getOversampling(PressureSensor))                                        //                                  //
    returnValue += (Time1*getOversampling(PressureSensor,true)) + Time2;      //                                  //
  if (getOversampling(HumiditySensor))                                        //                                  //
    returnValue += (Time1*getOversampling(HumiditySensor,true)) + Time2;      //                                  //
  return(returnValue);                                                        // Return inactive time setting     //
} // of method measurementTime()                                              //                                  //
/*******************************************************************************************************************
** Method getSensorData() returns the most recent temperature, humidity and pressure readings                     **
*******************************************************************************************************************/
void BME280_Class::getSensorData(int32_t &temp, int32_t &hum, int32_t &press){// get most recent readings         //
  readSensors();                                                              // Get compensated data from BME280 //
  temp  = _Temperature;                                                       // Copy global variable to parameter//
  hum   = _Humidity;                                                          //                                  //
  press = _Pressure;                                                          //                                  //
} // of method getSensorData()                                                //                                  //
/*******************************************************************************************************************
** Method reset() performs a device reset, as if it were powered down and back up again                           **
*******************************************************************************************************************/
void BME280_Class::reset() {                                                  // reset device                     //
   putData(BME280_SOFTRESET_REG,BME280_SOFTWARE_CODE);                        // writing code here resets device  //
   begin(_I2CAddress);                                                        // Start device again               //
} // of method reset()                                                        //                                  //