/**
 * @file RAK_EEPROM_I2C.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Driver to access EEPROM modules over I2C
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 * @remark This library is based on the [Adafruit FRAM I2C](https://github.com/adafruit/Adafruit_FRAM_I2C), but cleaned up from the BUS_IO overhead to make it leaner.
 */
#include <math.h>
#include <stdlib.h>

#include "RAK_EEPROM_I2C.h"

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
RAK_EEPROM_I2C::RAK_EEPROM_I2C(void) {}

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
bool RAK_EEPROM_I2C::begin(uint8_t addr, TwoWire *theWire)
{
	i2c_dev = theWire;
	_addr = addr;

	i2c_dev->begin();

	// A basic scanner, see if it ACK's
	i2c_dev->beginTransmission(_addr);
	if (i2c_dev->endTransmission() == 0)
	{
		return true;
	}

	return false;
}

/**************************************************************************/
/*!
    @brief  Writes a byte at the specific EEPROM address

    @param[in] addr
                The 16-bit address to write to in EEPROM memory
    @param[in] value
                The 8-bit value to write at addr
    @returns True on I2C command success, false on timeout or I2C failure
*/
/**************************************************************************/
bool RAK_EEPROM_I2C::write(uint16_t addr, uint8_t value)
{
	i2c_dev->beginTransmission(addr);
	i2c_dev->write(addr >> 8);
	i2c_dev->write(addr & 0xFF);
	i2c_dev->write(value);
	i2c_dev->endTransmission();

	// Wait until it acks!
	uint8_t timeout = 100;
	while (timeout--)
	{
		i2c_dev->beginTransmission(addr);
		if (i2c_dev->endTransmission() == 0)
		{
			return true;
		}
		delay(1);
	}

	// timed out :(
	return false;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value from the specified EEPROM address
    @param addr
                The 16-bit address to read from in EEPROM memory
    @returns    The 8-bit value retrieved at addr
*/
/**************************************************************************/
uint8_t RAK_EEPROM_I2C::read(uint16_t addr)
{
	i2c_dev->beginTransmission(addr);
	i2c_dev->write(addr >> 8);
	i2c_dev->write(addr & 0xFF);
	i2c_dev->endTransmission();

	size_t recv = i2c_dev->requestFrom(addr, (uint8_t)1);
	if (recv != 1)
	{
		return 0;
	}
	return i2c_dev->read();
}

/**************************************************************************/
/*!
    @brief  Writes multiple bytes at the specific EEPROM address

    @param[in] addr
                The 16-bit address to write to in EEPROM memory
    @param[in] buffer Pointer to buffer of bytes to write
    @param num How many bytes to write!
    @returns True on I2C command success, false on timeout or I2C failure
*/
/**************************************************************************/
bool RAK_EEPROM_I2C::write(uint16_t addr, uint8_t *buffer, uint16_t num)
{
	while (num--)
	{
		if (!write(addr++, buffer[0]))
		{
			return false;
		}
		buffer++;
	}
	return true;
}

/**************************************************************************/
/*!
    @brief  Reads multiple bytes from the specified EEPROM address
    @param addr
                The 16-bit address to read from in EEPROM memory
    @param buffer Pointer to buffer of bytes that will be filled!
    @param num How many bytes to write!
    @returns    The 8-bit value retrieved at addr
*/
/**************************************************************************/
bool RAK_EEPROM_I2C::read(uint16_t addr, uint8_t *buffer, uint16_t num)
{

	for (uint16_t i = 0; i < num; i++)
	{
		uint8_t buff[2] = {(uint8_t)(addr >> 8), (uint8_t)addr};

		if (!write(_addr, buff, 2))
			return false;
		if (!read(_addr, buff, 1))
			return false;

		buffer[i] = buff[0];

		addr++;
	}

	return true;
}
