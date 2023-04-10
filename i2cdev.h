/**
 * @file i2cdev.h
 *
 * ESP-IDF's component manages I2C ports, which makes it effective for working 
 * with RTOS.
 *
 * MIT License
 *
 * Copyright (c) 2023 phonght32
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct  I2C device handle structure.
 */
typedef struct i2cdev *i2cdev_handle_t;

/**
 * @struct  I2C device configuration structure.
 * 
 * @param   port I2C hardware port.
 * @param   mode I2C mode.
 * @param   sda_io_num SDA pin num.
 * @param   scl_io_num SCL pin num.
 * @param   clk_speed I2C frequency.
 */
typedef struct {
	i2c_port_t 		port;
	i2c_mode_t 		mode;
	gpio_num_t 		sda_io_num;
	gpio_num_t 		scl_io_num;
	uint32_t 		clk_speed;
} i2cdev_cfg_t;

/**
 * @brief 	Initialize I2C device module.
 * 
 * @param   config Pointer references to the I2C device configuration structure.
 *
 * @return  
 * 		- Pointer to the I2C device handle structure.
 * 		- NULL: Failed.
 */
i2cdev_handle_t i2cdev_init(i2cdev_cfg_t *config);

/**
 * @brief   Read data.
 * 
 * @param   handle I2C handle structure.
 * @param   reg_addr Register address.
 * @param   buf_recv Pointer references to the received buffer.
 * @param   size Data length.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed.
 */
esp_err_t i2cdev_read(i2cdev_handle_t handle, uint8_t reg_addr, uint8_t *buf_recv, size_t size);

/**
 * @brief   Write data.
 * 
 * @param   handle I2C handle structure.
 * @param   reg_addr Register address.
 * @param   buf_send Pointer references to the sended buffer.
 * @param   size Data length.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed.
 */
esp_err_t i2cdev_write(i2cdev_handle_t handle, uint8_t reg_addr, uint8_t *buf_send, size_t size);

/**
 * @brief   Declare target address.
 * 
 * @param   handle I2C handle structure.
 * @param   dev_addr Target address.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed.
 */
esp_err_t i2cdev_set_addr(i2cdev_handle_t handle, uint8_t dev_addr);

/**
 * @brief   Get target address.
 * 
 * @param   handle I2C handle structure. 
 * @param   dev_addr Pointer references to the target address.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t i2cdev_get_addr(i2cdev_handle_t handle, uint8_t *dev_addr);

/**
 * @brief   Destroy I2C device module.
 * 
 * @param   handle I2C handle structure. 
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t i2cdev_destroy(i2cdev_handle_t handle);


#ifdef __cplusplus
}
#endif

#endif /* _I2CDEV_H_ */