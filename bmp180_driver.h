#ifndef __BMP180_DRIVER__
#define __BMP180_DRIVER__

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define BMP180_I2C_ADDR 0x77
#define BMP180_CHIP_ID 0x55
#define BMP180_READ_TEMP_CMD 0x2E
#define BMP180_READ_PRESSURE_CMD 0x34
#define BMP180_MSR_SCO_MASK (1 << 5)

#define TEMP_MSR_TIME 4.5
#define LP_MSR_TIME 4.5
#define STANDARD_MSR_TIME 7.5
#define HR_MSR_TIME 13.5
#define ULTRA_HR_MSR_TIME 25.5

enum bmp180_reg {
	REG_CALIB0 = 0xAA,
	REG_CALIB21 = 0xBF,
	REG_ID = 0xD0,
	REG_RESET = 0xE0,
	REG_CTRL_MEAS = 0xF4,
	REG_OUT_MSB = 0xF6,
	REG_OUT_LSB = 0xF7,
	REG_OUT_XLSB = 0xF8,
};

typedef enum {
	LOW_POWER = 0,
	STANDARD,
	HIGH_RES,
	ULTRA_HR
} bmp180_mode_t;

typedef enum {
	STATE_MSR_TEMP,
	STATE_GET_TEMP,
	STATE_MSR_PRES,
	STATE_GET_PRES
} State;

struct bmp180 {
	i2c_master_dev_handle_t dev_handle;
	bmp180_mode_t oss;
	State state;

	int32_t raw_pressure;
	int32_t raw_temp;
	int32_t true_pressure;
	float true_temp;

	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;

	int32_t B5;
};

esp_err_t bmp180_init(struct bmp180 *sensor, i2c_master_dev_handle_t *dev_handle,
					bmp180_mode_t oss);

esp_err_t bmp180_measure(struct bmp180 *sensor);

esp_err_t bmp180_init_height(struct bmp180 *sensor);

int32_t bmp180_get_rel_height(struct bmp180 *sensor);

#endif // __BMP180_DRIVER__

