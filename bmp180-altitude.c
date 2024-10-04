#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "driver/i2c_master.h"
#include "esp_check.h"

#include "bmp180_driver.h"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */

static const char* TAG = "Main Application";

void app_main(void)
{
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = -1,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = BMP180_I2C_ADDR,
		.scl_speed_hz = 100000,
	};

	i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

	struct bmp180 sensor;
	esp_err_t err = bmp180_init(&sensor, &dev_handle, STANDARD);
	err |= bmp180_init_height(&sensor);
	if (err != ESP_OK) {
		while (1) {
			;;
		}
	}

	while (1) {
		bmp180_measure(&sensor);
		if (sensor.state == STATE_MSR_TEMP) {
			int32_t h = bmp180_get_rel_height(&sensor);
			ESP_LOGI(TAG, "Height: %ld\n", h);
		}
		if (sensor.state == STATE_GET_TEMP)
			vTaskDelay(pdMS_TO_TICKS(TEMP_MSR_TIME));
		else if (sensor.state == STATE_GET_PRES)
			vTaskDelay(pdMS_TO_TICKS(STANDARD_MSR_TIME));
		/* ESP_LOGI(TAG, "Temp: %f, pres: %ld\n", sensor.true_temp, sensor.true_pressure); */
		if (err != ESP_OK) {
			ESP_LOGI(TAG, "Error!!!!\n");
		}
	}
}

