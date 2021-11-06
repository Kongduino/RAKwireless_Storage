/**
 * @file RAK_FRAM_I2C.cpp
 * @author KTOWN (Adafruit Industries)
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 * @remark This library is based on the [Adafruit FRAM I2C](https://github.com/adafruit/Adafruit_FRAM_I2C), but cleaned up from the BUS_IO overhead to make it leaner.
 */
#include <math.h>
#include <stdlib.h>

#include "RAK_FRAM_I2C.h"

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
RAK_FRAM_I2C::RAK_FRAM_I2C(void) { _framInitialised = false; }

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  addr
 *            The I2C address to be used.
 *    @param  theWire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool RAK_FRAM_I2C::begin(uint8_t addr, TwoWire *theWire)
{
	RAK_EEPROM_I2C::begin(addr, theWire);

	// the MB85 has a secondary address too!
	i2c_dev2 = theWire;
	_addr = addr;

	i2c_dev2->begin();

	// A basic scanner, see if it ACK's
	i2c_dev2->beginTransmission(_addr);
	if (i2c_dev2->endTransmission() != 0)
	{
		return false;
	}

	/* Make sure we're actually connected */
	uint16_t manufID, prodID;
	getDeviceID(&manufID, &prodID);
	if (manufID != 0x00A)
	{
		Serial.print("Unexpected Manufacturer ID: 0x");
		Serial.println(manufID, HEX);
		return false;
	}
	if (prodID != 0x510)
	{
		Serial.print("Unexpected Product ID: 0x");
		Serial.println(prodID, HEX);
		return false;
	}

	/* Everything seems to be properly initialised and connected */
	_framInitialised = true;

	return true;
}

/**************************************************************************/
/*!
    @brief  Reads the Manufacturer ID and the Product ID frm the IC

    @param[out]  manufacturerID
                  The 12-bit manufacturer ID (Fujitsu = 0x00A)
    @param[out]  productID
                  The memory density (bytes 11..8) and proprietary
                  Product ID fields (bytes 7..0). Should be 0x510 for
                  the MB85RC256V.
*/
/**************************************************************************/
void RAK_FRAM_I2C::getDeviceID(uint16_t *manufacturerID,
							   uint16_t *productID)
{
	uint8_t buff[3] = {(uint8_t)(_addr * 2), 0, 0};
	write(_addr, buff[0]);
	read(_addr, buff, 3);

	/* Shift values to separate manuf and prod IDs */
	/* See p.10 of
   * http://www.fujitsu.com/downloads/MICRO/fsa/pdf/products/memory/fram/MB85RC256V-DS501-00017-3v0-E.pdf
   */
	*manufacturerID = (buff[0] << 4) + (buff[1] >> 4);
	*productID = ((buff[1] & 0x0F) << 8) + buff[2];
}
