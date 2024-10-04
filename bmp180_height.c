#include "bmp180_driver.h"
#include "esp_log.h"

#define INIT_PRESSURE_SAMPLES 50
#define GRAV_CONSTANT_MPS2 9.81f
#define AIR_DENSITY_KG_M3 1.225f

static const char* TAG = "BMP180 Height Module";

static int32_t initial_pressure;

esp_err_t bmp180_init_height(struct bmp180 *sensor) {
	esp_err_t err = ESP_OK;
	uint16_t samples = 0;
	while (samples < INIT_PRESSURE_SAMPLES) {
		/* Continuously poll the sensor for the fastest initialization */
		err = bmp180_measure(sensor);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Height initialization error\n");
			return err;
		}
		if (sensor->state == STATE_MSR_PRES) {
			initial_pressure += sensor->true_pressure;
			samples++;
		}
	}
	initial_pressure /= INIT_PRESSURE_SAMPLES;
	return err;
}

int32_t bmp180_get_rel_height(struct bmp180 *sensor) {
	int32_t height = -(sensor->true_pressure - initial_pressure) / (GRAV_CONSTANT_MPS2 * AIR_DENSITY_KG_M3);
	return height;
}

