/**
 * @file i2cdev.c
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

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2cdev.h"

#define I2C_TIMEOUT                 3000						/*!< I2C time out in milisecond */
#define ACK_VAL                     0x0                         /*!< I2C ack value */
#define NACK_VAL                    0x1                         /*!< I2C nack value */
#define ACK_CHECK_EN                0x1                         /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0                         /*!< I2C master will not check ack from slave */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */

/**
 * @macro 	Mutex macros.
 */
#define mutex_lock(x)					while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 				xSemaphoreGive(x)
#define mutex_create()					xSemaphoreCreateMutex()
#define mutex_destroy(x) 				vQueueDelete(x)

/**
 * @macro 	I2C device check.
 */
#define I2CDEV_CHECK(a, str, action) if(!(a)) {								\
	ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);	\
	action;																	\
}

/**
 * @brief   Module tag that is displayed in ESP_LOG.
 */
static const char *TAG = "I2CDEV";

/**
 * @struct 	I2C device structure.
 *
 * @param   port I2C hardware port.
 * @param   dev_addr Target address.
 * @param   lock Mutex.
 * @param   is_run Running status.
 */
typedef struct i2cdev {
	i2c_port_t 				port;
	uint8_t 				dev_addr;
	SemaphoreHandle_t 		lock;
	bool 					is_run;
} i2cdev_t;

/**
 * @func    i2cdev_init
 */
i2cdev_handle_t i2cdev_init(i2cdev_cfg_t *config)
{
	I2CDEV_CHECK(config, "error config null", return NULL);

	i2cdev_handle_t handle = calloc(1, sizeof(i2cdev_t));
	I2CDEV_CHECK(handle, "i2cdev allocate memory error", return NULL);

	i2c_config_t i2c_config = {
		.mode = config->mode,
		.scl_io_num = config->scl_io_num,
		.sda_io_num = config->sda_io_num,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = config->clk_speed,
	};
	ESP_LOGI(TAG, "port: %d, sda: %d, scl: %d, speed: %d",
	         (unsigned int)config->port,
	         (unsigned int)config->sda_io_num,
	         (unsigned int)config->scl_io_num,
	         (unsigned int)config->clk_speed);

	int ret;

	ret = i2c_param_config(config->port, &i2c_config);
	I2CDEV_CHECK(!ret, "config param error", {free(handle); return NULL;});

	ret = i2c_driver_install(config->port, config->mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	I2CDEV_CHECK(!ret, "driver install error", {free(handle); return NULL;});

	handle->port = config->port;
	handle->lock = mutex_create();
	handle->is_run = true;

	return handle;
}

/**
 * @func    i2cdev_read
 */
esp_err_t i2cdev_read(i2cdev_handle_t handle, uint8_t reg_addr, uint8_t *buf_recv, size_t size)
{
	I2CDEV_CHECK(handle, "error handle null", return ESP_ERR_INVALID_ARG);
	I2CDEV_CHECK(buf_recv, "error buffer receive null", return ESP_ERR_INVALID_ARG);

	if (size == 0) {
		return ESP_OK;
	}

	mutex_lock(handle->lock);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (handle->dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (handle->dev_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	if (size > 1) {
		i2c_master_read(cmd, buf_recv, size - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, buf_recv + size - 1, NACK_VAL);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(handle->port, cmd, I2C_TIMEOUT / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	mutex_unlock(handle->lock);

	return ESP_OK;
}

/**
 * @func    i2cdev_write
 */
esp_err_t i2cdev_write(i2cdev_handle_t handle, uint8_t reg_addr, uint8_t *buf_send, size_t size)
{
	I2CDEV_CHECK(handle, "error handle null", return ESP_ERR_INVALID_ARG);
	I2CDEV_CHECK(buf_send, "error buffer send null", return ESP_ERR_INVALID_ARG);

	int ret;

	mutex_lock(handle->lock);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (handle->dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_write(cmd, buf_send, size, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(handle->port, cmd, I2C_TIMEOUT / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	mutex_unlock(handle->lock);

	return ret;
}

/**
 * @func    i2cdev_set_addr
 */
esp_err_t i2cdev_set_addr(i2cdev_handle_t handle, uint8_t dev_addr)
{
	I2CDEV_CHECK(handle, "error handle null", return ESP_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	handle->dev_addr = dev_addr;
	mutex_unlock(handle->lock);

	return ESP_OK;
}

/**
 * @func    i2cdev_get_addr
 */
esp_err_t i2cdev_get_addr(i2cdev_handle_t handle, uint8_t *dev_addr)
{
	I2CDEV_CHECK(handle, "error handle null", return ESP_ERR_INVALID_ARG);
	I2CDEV_CHECK(dev_addr, "error device address null", return ESP_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	*dev_addr = handle->dev_addr;
	mutex_unlock(handle->lock);

	return ESP_OK;
}

/**
 * @func    i2cdev_destroy
 */
esp_err_t i2cdev_destroy(i2cdev_handle_t handle)
{
	I2CDEV_CHECK(handle, "error handle null", return ESP_ERR_INVALID_ARG);
	mutex_destroy(handle->lock);
	free(handle);

	return ESP_OK;
}