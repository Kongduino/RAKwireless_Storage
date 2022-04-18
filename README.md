# RAKwireless EEPROM & FRAM Library

This is a library for our RAKwireless WisBlock Storage modules.

I2C device parts in this library is based on [Adafruit FRAM I2C](https://github.com/adafruit/Adafruit_FRAM_I2C), but cleaned up from the BUS_IO overhead to make it leaner.    
SPI device parts in this library is based on [Adafruit SPIFlash](https://github.com/adafruit/Adafruit_SPIFlash), but cleaned up by removing the filesystem part to make it leaner.    

I2C device parts are covered by BSD license by Adafruit => LICENSE file    
SPI device parts are covered by MIT license by Adafruit => header of each file    


# Changelog

2022-04-18 V1.0.2
   - Fix uppercase type for include.

2022-04-18 V1.0.1
   - Add support for SPI based Flash and FRAM modules.

2021-11-06 V1.0.0
   - First release