#include "esp_log.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "i2cdev.h"

#define I2C_TIMEOUT                 3000
#define ACK_VAL                     0x0                         /*!< I2C ack value */
#define NACK_VAL                    0x1                         /*!< I2C nack value */
#define ACK_CHECK_EN                0x1                         /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0                         /*!< I2C master will not check ack from slave */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */

#define mutex_lock(x)					while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 				xSemaphoreGive(x)
#define mutex_create()					xSemaphoreCreateMutex()
#define mutex_destroy(x) 				vQueueDelete(x)

#define I2CDEV_CHECK(a, str, action) if(!(a)) {								\
	ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);	\
	action;																	\
}

static const char *TAG = "I2CDEV";

typedef struct i2cdev {
	i2c_port_t 				port;
	uint8_t 				dev_addr;
	SemaphoreHandle_t 		lock;
	bool 					is_run;
} i2cdev_t;

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
	ESP_LOGD(TAG, "port: %d, sda: %d, scl: %d, speed: %d", config->port, config->sda_io_num, config->scl_io_num, config->clk_speed);

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

esp_err_t i2cdev_set_addr(i2cdev_handle_t handle, uint8_t dev_addr)
{
	I2CDEV_CHECK(handle, "error handle null", return ESP_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	handle->dev_addr = dev_addr;
	mutex_unlock(handle->lock);

	return ESP_OK;
}

esp_err_t i2cdev_get_addr(i2cdev_handle_t handle, uint8_t *dev_addr)
{
	I2CDEV_CHECK(handle, "error handle null", return ESP_ERR_INVALID_ARG);
	I2CDEV_CHECK(dev_addr, "error device address null", return ESP_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	*dev_addr = handle->dev_addr;
	mutex_unlock(handle->lock);

	return ESP_OK;
}