/**
 * @file RAK_FRAM_I2C.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Includes and defines for RAK_FRAM_I2C class
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 * @remark This library is based on the [Adafruit FRAM I2C](https://github.com/adafruit/Adafruit_FRAM_I2C), but cleaned up from the BUS_IO overhead to make it leaner.
 */
#ifndef _RAK_FRAM_I2C_H_
#define _RAK_FRAM_I2C_H_

#include <RAK_EEPROM_I2C.h>

#define MB85RC_DEFAULT_ADDRESS \
	(0x50) ///<* 1010 + A2 + A1 + A0 = 0x50 default */
#define MB85RC_SECONDARY_ADDRESS \
	(0x7C) ///< secondary ID for manufacture id info

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            I2C FRAM chips
 */
class RAK_FRAM_I2C : public RAK_EEPROM_I2C
{
public:
	RAK_FRAM_I2C(void);

	bool begin(uint8_t addr = MB85RC_DEFAULT_ADDRESS, TwoWire *theWire = &Wire);
	void getDeviceID(uint16_t *manufacturerID, uint16_t *productID);

private:
	TwoWire *i2c_dev2 = NULL;
	boolean _framInitialised;
};

#endif
