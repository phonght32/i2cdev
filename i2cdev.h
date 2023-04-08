#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c.h"

typedef struct i2cdev *i2cdev_handle_t;

typedef struct {
	i2c_port_t 		port;
	i2c_mode_t 		mode;
	gpio_num_t 		sda_io_num;
	gpio_num_t 		scl_io_num;
	uint32_t 		clk_speed;
} i2cdev_cfg_t;

i2cdev_handle_t i2cdev_init(i2cdev_cfg_t *config);
esp_err_t i2cdev_read(i2cdev_handle_t handle, uint8_t reg_addr, uint8_t *buf_recv, size_t size);
esp_err_t i2cdev_write(i2cdev_handle_t handle, uint8_t reg_addr, uint8_t *buf_send, size_t size);
esp_err_t i2cdev_set_addr(i2cdev_handle_t handle, uint8_t dev_addr);
esp_err_t i2cdev_get_addr(i2cdev_handle_t handle, uint8_t *dev_addr);
esp_err_t i2cdev_destroy(i2cdev_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _I2CDEV_H_ */