#include "bmp180_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"

#define CALIB_REG_LEN REG_CALIB21 - REG_CALIB0  + 1

static const char* TAG = "BMP180 Driver";

esp_err_t bmp180_write_reg(struct bmp180 *sensor, uint8_t reg, uint8_t val) {
	uint8_t tmp[2] = { reg, val };
	return i2c_master_transmit(sensor->dev_handle, tmp, 2, -1);
}

esp_err_t bmp180_read_reg(struct bmp180 *sensor, uint8_t reg, uint8_t *val) {
	return i2c_master_transmit_receive(sensor->dev_handle, &reg, 1, val, 1, -1);
}

esp_err_t bmp180_bulk_read(struct bmp180 *sensor, uint8_t start_reg, uint8_t len, uint8_t *out) {
	return i2c_master_transmit_receive(sensor->dev_handle, &start_reg, 1, out, len, -1);
}

esp_err_t bmp180_init(struct bmp180 *sensor, i2c_master_dev_handle_t *dev_handle,
					bmp180_mode_t oss) {
	esp_err_t err;
	if (dev_handle == NULL || sensor == NULL)
		return ESP_ERR_INVALID_ARG;

	sensor->dev_handle = *dev_handle;
	sensor->oss = oss;
	sensor->state = STATE_MSR_TEMP;
	sensor->true_temp = 0;
	sensor->true_pressure = 0;

	uint8_t id;
	err = bmp180_read_reg(sensor, REG_ID, &id);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to communicate with BMP180\n");
		return err;
	}
	if (id != BMP180_CHIP_ID) {
		ESP_LOGE(TAG, "BMP180 chip ID is different from retrieved\n");
		return err;
	}

	uint8_t tmp[CALIB_REG_LEN];
	err = bmp180_bulk_read(sensor, REG_CALIB0, CALIB_REG_LEN, tmp);
	sensor->AC1 = tmp[0] << 8 | tmp[1];
	sensor->AC2 = tmp[2] << 8 | tmp[3];
	sensor->AC3 = tmp[4] << 8 | tmp[5];
	sensor->AC4 = tmp[6] << 8 | tmp[7];
	sensor->AC5 = tmp[8] << 8 | tmp[9];
	sensor->AC6 = tmp[10] << 8 | tmp[11];
	sensor->B1 = tmp[12] << 8 | tmp[13];
	sensor->B2 = tmp[14] << 8 | tmp[15];
	sensor->MB = tmp[16] << 8 | tmp[17];
	sensor->MC = tmp[18] << 8 | tmp[19];
	sensor->MD = tmp[20] << 8 | tmp[21];

	/* Initialize true_temp and true_pressure */
	do {
		bmp180_measure(sensor);
	} while (sensor->state != STATE_MSR_TEMP);

	return err;
}

uint8_t bmp180_check_ready(struct bmp180 *sensor) {
	uint8_t ctrl_val;
	bmp180_read_reg(sensor, REG_CTRL_MEAS, &ctrl_val);
	if (ctrl_val & BMP180_MSR_SCO_MASK) {
		return 0;
	}
	return 1;
}

esp_err_t bmp180_start_ut(struct bmp180 *sensor);
esp_err_t bmp180_get_ut(struct bmp180 *sensor);
esp_err_t bmp180_start_up(struct bmp180 *sensor);
esp_err_t bmp180_get_up(struct bmp180 *sensor);

esp_err_t bmp180_change_state(struct bmp180 *sensor) {
	esp_err_t err = ESP_OK;
	switch (sensor->state) {
		case STATE_MSR_TEMP:
			err = bmp180_start_ut(sensor);
			sensor->state = STATE_GET_TEMP;
			break;
		case STATE_GET_TEMP:
			if (!bmp180_check_ready(sensor)) {
				return ESP_ERR_NOT_FINISHED;
			}
			err = bmp180_get_ut(sensor);
			sensor->state = STATE_MSR_PRES;
			break;
		case STATE_MSR_PRES:
			err = bmp180_start_up(sensor);
			sensor->state = STATE_GET_PRES;
			break;
		case STATE_GET_PRES:
			if (!bmp180_check_ready(sensor)) {
				return ESP_ERR_NOT_FINISHED;
			}
			err = bmp180_get_up(sensor);
			sensor->state = STATE_MSR_TEMP;
			break;
	}
	return err;
}

esp_err_t bmp180_start_ut(struct bmp180 *sensor) {
	esp_err_t err;
	uint8_t reg = REG_CTRL_MEAS;
	uint8_t command = BMP180_READ_TEMP_CMD;
	err = bmp180_write_reg(sensor, reg, command);
	return err;
}

esp_err_t bmp180_get_ut(struct bmp180 *sensor) {
	esp_err_t err;
	uint8_t raw_temp[2];
	err = bmp180_bulk_read(sensor, REG_OUT_MSB, 2, raw_temp);
	sensor->raw_temp = raw_temp[0] << 8 | raw_temp[1];
	return err;
}

esp_err_t bmp180_start_up(struct bmp180 *sensor) {
	esp_err_t err;
	uint8_t reg = REG_CTRL_MEAS;
	uint8_t command = BMP180_READ_PRESSURE_CMD | (sensor->oss << 6);
	err = bmp180_write_reg(sensor, reg, command);
	return err;
}

esp_err_t bmp180_get_up(struct bmp180 *sensor) {
	esp_err_t err;
	uint8_t raw_pressure[3];
	err = bmp180_bulk_read(sensor, REG_OUT_MSB, 3, raw_pressure);
	sensor->raw_pressure = ((raw_pressure[0] << 16) | (raw_pressure[1] << 8) | raw_pressure[2]) >> (8 - sensor->oss);
	return err;
}

esp_err_t bmp180_measure(struct bmp180 *sensor) {
	esp_err_t err;
	int32_t X1, X2, B5;
	int32_t X3, B3, B6, p;
	uint32_t B4, B7;

	err = bmp180_change_state(sensor);
	if (err == ESP_ERR_NOT_FINISHED)
		return ESP_OK;

	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to communicate with device\n");
		return err;
	}
	if (sensor->state == STATE_MSR_PRES) {
		X1 = ((sensor->raw_temp - sensor->AC6) * sensor->AC5) >> 15 ;
		X2 = (sensor->MC << 11) / (X1 + sensor->MD);
		B5 = X1 + X2;
		sensor->B5 = B5;
		sensor->true_temp = ((B5 + 8) >> 4) * 0.1;
	}
	else if (sensor->state == STATE_MSR_TEMP) {
		B6 = sensor->B5 - 4000;
		X1 = (sensor->B2 * ((B6 * B6) >> 12)) >> 11;
		X2 = (sensor->AC2 * B6) >> 11;
		X3 = X1 + X2;

		B3 = ((((int32_t)sensor->AC1 * 4 + X3) << sensor->oss) + 2) >> 2;
		X1 = (sensor->AC3 * B6) >> 13;
		X2 = (sensor->B1 * ((B6 * B6) >> 12)) >> 16;
		X3 = ((X1 + X2) + 2) >> 2;
		B4 = (sensor->AC4 * (uint32_t)(X3 + 32768)) >> 15;
		B7 = ((uint32_t)sensor->raw_pressure - B3) * (50000UL >> sensor->oss);

		if (B7 < 0x80000000UL)
			p = (B7 << 1) / B4;
		else
			p = (B7 / B4) << 1;

		X1 = (p >> 8) * (p >> 8);
		X1 = (X1 * 3038) >> 16;
		X2 = (-7357 * p) >> 16;
		p += (X1 + X2 + 3791) >> 4;

		sensor->true_pressure = p;
	}
	return err;
}


