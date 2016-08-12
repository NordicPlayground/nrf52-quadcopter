// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
//
// Nordic Semiconductors nrf51 code by August Bering <augustbering@yahoo.com>, tested with MPU6050 only
// readTimeout not implemented.

// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//


/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "i2cdev.h"

#include <stdio.h>
#include <string.h>

#define I2CDEV_NO_MEM_ADDR  0xFF

static nrf_drv_twi_t m_twi=NRF_DRV_TWI_INSTANCE(0);

#define SCL_PIN 13
#define SDA_PIN 12

void i2cdev_initialize(void) {
	 ret_code_t err_code;

    const nrf_drv_twi_config_t config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    err_code = nrf_drv_twi_init(&m_twi, &config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
}

/** Enable or disable I2C
 * @param isEnabled true = enable, false = disable
 */
void i2cdev_enable(bool isEnabled) {
  
	if (isEnabled)
		nrf_drv_twi_enable(&m_twi);
	else
		nrf_drv_twi_disable(&m_twi);

}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t i2cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = i2cdev_readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t i2cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = i2cdev_readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t i2cdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    return i2cdev_readBytes(devAddr, regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t i2cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
	
	//Used for MS5611 Pressure sensor
	if(regAddr != I2CDEV_NO_MEM_ADDR) {
		nrf_drv_twi_tx(&m_twi,devAddr,&regAddr,1,true);
	}
	
	ret_code_t r= nrf_drv_twi_rx(&m_twi,devAddr,data,length);

	return r==NRF_SUCCESS;
  
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool i2cdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2cdev_readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2cdev_writeByte(devAddr, regAddr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool i2cdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (i2cdev_readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return i2cdev_writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool i2cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    uint8_t w2_data[2];
		uint8_t length = 2;
		
		if(regAddr != I2CDEV_NO_MEM_ADDR) {
			w2_data[0] = regAddr;
			w2_data[1] = data;
		} else {
			w2_data[0] = data;
			w2_data[1] = data;
			length = 1;
		}
    return NRF_SUCCESS==nrf_drv_twi_tx(&m_twi,devAddr,w2_data,length,false);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool i2cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
	
	/*
	if (NRF_SUCCESS!=nrf_drv_twi_tx(&m_twi,devAddr,&regAddr,1,true))
		return false;
	return NRF_SUCCESS==nrf_drv_twi_tx(&m_twi,devAddr,data,length,false);
	*/
	
	//TODO: This could be improved...
	uint8_t buffer[32];
	buffer[0] = regAddr;
	uint8_t i = 1;
	while(i < (length + 1))
		buffer[i++] = *data++;
	
	return NRF_SUCCESS==nrf_drv_twi_tx(&m_twi,devAddr,buffer,length+1,false);
}
/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool i2cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return i2cdev_writeWords(devAddr, regAddr, 1, &data);
}
/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool i2cdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data){
	if (NRF_SUCCESS!=nrf_drv_twi_tx(&m_twi,devAddr,&regAddr,1,true))
		return false;

	for (int i=0;i<length;i++){
		uint8_t d[2];
		d[0]=data[i]>>8;
		d[1]=data[i]&0xff;
	bool pending = i < length - 1;
	if (NRF_SUCCESS != nrf_drv_twi_tx(&m_twi, devAddr, d, 2, pending))
			return false;
	}
	return true;
}
