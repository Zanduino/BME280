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
#include "BME280.h"                                                          // Include the header definition    //
                                                                              //                                  //
/*******************************************************************************************************************
** Class Constructor instantiates the class                                                                       **
*******************************************************************************************************************/
BME280_Class::BME280_Class()  {} // of class constructor                    //                                  //
/*******************************************************************************************************************
** Class Destructor currently does nothing and is included for compatibility purposes                             **
*******************************************************************************************************************/
BME280_Class::~BME280_Class() {} // of class destructor                     //                                  //
/*******************************************************************************************************************
** Method begin starts I2C communications with the device, using a default address if one is not specified and    **
** return true if the device has been detected and false if it was not                                            **
*******************************************************************************************************************/
bool BME280_Class::begin(uint8_t I2CAddress = BME280_ADDRESS ) {              // Start I2C communications         //
  Wire.begin();                                                               // Start I2C as master device       //
  _I2CAddress = I2CAddress;                                                   // Store actual I2C address to use  //
  Wire.beginTransmission(_I2CAddress);                                        // Address the BME280               //
  delay(BME280_I2C_DELAY);                                                    // Give the BME280 time to process  //
  uint8_t errorCode = Wire.endTransmission();                                 // See if there's a device present  //
  if (errorCode == 0) {                                                       // If we have a BME280              //

  } // of if-then device detected                                             //                                  //
  else return false;                                                          // return error if no device found  //
  return true;                                                                // return success                   //
} // of method begin()                                                        //                                  //
/*******************************************************************************************************************
** Method readByte reads 1 byte from the specified address                                                        **
*******************************************************************************************************************/
uint8_t BME280_Class::readByte(const uint8_t addr) {                          //                                  //
  Wire.beginTransmission(_I2CAddress);                                        // Address the I2C device           //
  Wire.write(addr);                                                           // Send the register address to read//
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
  delayMicroseconds(BME280_I2C_DELAY);                                        // delay required for sync          //
  Wire.requestFrom(_I2CAddress, (uint8_t)1);                                  // Request 1 byte of data           //
  while(!Wire.available());                                                   // Wait until the byte is ready     //
  return Wire.read();                                                         // read it and return it            //
} // of method readByte()                                                     //                                  //
/*******************************************************************************************************************
** Method writeByte write 1 byte to the specified address                                                         **
*******************************************************************************************************************/
void BME280_Class::writeByte(const uint8_t addr, const uint8_t data) {        //                                  //
  Wire.beginTransmission(_I2CAddress);                                        // Address the I2C device           //
  Wire.write(addr);                                                           // Send the register address to read//
  Wire.write(data);                                                           // Send the register address to read//
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
} // of method writeByte()                                                    //                                  //
/*******************************************************************************************************************
** Method writeI2C writes n-bytes to the specified address                                                        **
*******************************************************************************************************************/
bool BME280_Class::writeI2C(const uint8_t addr, uint8_t *pdata,               //                                  //
                             const uint8_t bytesToWrite) {                    //                                  //
  Wire.beginTransmission(_I2CAddress);                                        // Address the I2C device           //
  Wire.write(addr);                                                           // Send the register address to read//
  uint8_t i = bytesToWrite;                                                   // loop index                       //
  while (i--) {                                                               // post-decrement the counter       //
    Wire.write((uint8_t)pdata[0]);                                            // Write the byte addressed by ptr  //
    pdata++;                                                                  // increment pointer to next address//
  } // while we still have bytes to write                                     //                                  //
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
} // of method writeI2C()                                                     //                                  //
/*******************************************************************************************************************
** Method readI2C reads n-bytes from the specified address                                                        **
*******************************************************************************************************************/
uint8_t BME280_Class::readI2C(const uint8_t addr, uint8_t *pdata,             //                                  //
                           const uint8_t bytesToRead) {                       //                                  //
  Wire.beginTransmission(_I2CAddress);                                        // Address the I2C device           //
  Wire.write(addr);                                                           // Send the register address to read//
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
  delayMicroseconds(BME280_I2C_DELAY);                                        // delay required for sync          //
  Wire.requestFrom(_I2CAddress, (uint8_t)bytesToRead);                        // Request 1 byte of data           //
  while(!Wire.available());                                                   // Wait until the first byte ready  //
  uint8_t i = bytesToRead;                                                    // loop index                       //
  while (i--) {                                                               // post-decrement the counter       //
    pdata[0] = Wire.read();                                                   // Read byte to pointer address     //
    pdata++;                                                                  // increment pointer to next address//
  } // while we still have bytes to write                                     //                                  //
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
} // of method readI2C()                                                      //                                  //
