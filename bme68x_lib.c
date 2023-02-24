/**
  ******************************************************************************
  * @file           : bme68x_lib.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Dec 6, 2022
  * @brief          : todo: write brief
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2022 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "bme68x_lib.h"
#include "esp_log.h"
#include "esp_timer.h"

/* Private macro -------------------------------------------------------------*/
/* Maximum transaction size. Field size 17 x 3 */
#define BME68X_MAX_READ_LENGTH 51

#define BME68X_I2C_BUFFER_SIZE 64

#define NOP() asm volatile ("nop")

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const static char * TAG = "bme688";

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Function that implements the default microsecond delay callback
 *
 * @param period_us : Duration of the delay in microseconds
 * @param intf_ptr  : Pointer to the interface descriptor
 */
static void delay_us(uint32_t period_us, void * intf_ptr);

/**
 * @brief Function that implements the default SPI write transaction
 *
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param intf_ptr : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t spi_write(uint8_t reg_addr, const uint8_t * reg_data,
		uint32_t length, void * intf_ptr);

/**
 * @brief Function that implements the default SPI read transaction
 *
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to be read from the sensor
 * @param length   : Length of the transfer
 * @param intf_ptr : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t spi_read(uint8_t reg_addr, uint8_t * reg_data, uint32_t length,
		void * intf_ptr);

static esp_err_t i2c_init(i2c_t * i2c);

/**
 * @brief Function that implements the default I2C write transaction
 *
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param intf_ptr : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t i2c_write(uint8_t reg_addr, const uint8_t * reg_data,
		uint32_t length, void * intf_ptr);

/**
 * @brief Function that implements the default I2C read transaction
 *
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param intf_ptr : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t i2c_read(uint8_t reg_addr, uint8_t * reg_data, uint32_t length,
		void * intf_ptr);

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Function to initialize a BME68x instance
 */
esp_err_t bme68x_lib_init(bme68x_lib_t * const me, void * arg, bme68x_intf_t intf) {
	ESP_LOGI(TAG, "Initializing BME688 instance...");

	me->status = BME68X_OK;
	memset(&me->bme6, 0, sizeof(me->bme6));
	memset(&me->conf, 0, sizeof(me->conf));
	memset(&me->heatr_conf, 0, sizeof(me->heatr_conf));
	memset(&me->sensor_data, 0, sizeof(me->sensor_data));
	me->n_fields = 0;
	me->i_fields = 0;
	me->last_op_mode = BME68X_SLEEP_MODE;

	/*  */
	if (intf == BME68X_I2C_INTF) {
		me->comm.i2c = (i2c_t *)arg;
		ESP_ERROR_CHECK(i2c_init(me->comm.i2c));
		me->bme6.read = i2c_read;
		me->bme6.write = i2c_write;
	}
	else {
		me->comm.spi = (spi_t *)arg;
		me->bme6.read = spi_read;
		me->bme6.write = spi_write;
	}

	me->bme6.intf = intf;
	me->bme6.delay_us = delay_us;
	me->bme6.intf_ptr = &me->comm;
	me->bme6.amb_temp = 25;

	me->status = bme68x_init(&me->bme6);

	return me->status > 0 ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Function to read a register
 */
uint8_t bme68x_lib_read_single_reg(bme68x_lib_t * const me, uint8_t reg_addr) {
	uint8_t reg_data;
	bme68x_lib_read_multiple_reg(me, reg_addr, &reg_data, 1);

	return reg_data;
}

/**
 * @brief Function to read multiple registers
 */
void bme68x_lib_read_multiple_reg(bme68x_lib_t * const me, uint8_t reg_addr, uint8_t * reg_data, uint32_t length) {
	me->status = bme68x_get_regs(reg_addr, reg_data, length, &me->bme6);
}

/**
 * @brief Function to write data to a register
 */
void bme68x_lib_write_single_reg(bme68x_lib_t * const me, uint8_t reg_addr, uint8_t reg_data) {
	me->status = bme68x_set_regs(&reg_addr, &reg_data, 1, &me->bme6);
}

/**
 * @brief Function to write multiple registers
 */
void bme68x_lib_write_multiple_reg(bme68x_lib_t * const me, uint8_t * reg_addr, const uint8_t * reg_data, uint32_t length){
	me->status = bme68x_set_regs(reg_addr, reg_data, length, &me->bme6);
}

/**
 * @brief Function to trigger a soft reset
 */
void bme68x_lib_soft_reset(bme68x_lib_t * const me) {
	me->status = bme68x_soft_reset(&me->bme6);
}

/**
 * @brief Function to set the ambient temperature for better configuration
 */
void bme68x_lib_set_ambient_temp(bme68x_lib_t * const me, int8_t temp) {
	me->bme6.amb_temp = temp;
}

/**
 * @brief Function to get the measurement duration in microseconds
 */
uint32_t bme68x_lib_get_meas_dur(bme68x_lib_t * const me, uint8_t op_mode) {
	if (op_mode == BME68X_SLEEP_MODE) {
		op_mode = me->last_op_mode;
	}

	return bme68x_get_meas_dur(op_mode, &me->conf, &me->bme6);
}

/**
 * @brief Function to set the operation mode
 */
void bme68x_lib_set_op_mode(bme68x_lib_t * const me, uint8_t op_mode) {
	me->status = bme68x_set_op_mode(op_mode, &me->bme6);

	if ((me->status == BME68X_OK) && (op_mode != BME68X_SLEEP_MODE)) {
		me->last_op_mode = op_mode;
	}
}

/**
 * @brief Function to get the operation mode
 */
uint8_t bme68x_lib_get_op_mode(bme68x_lib_t * const me) {
	uint8_t op_mode;

	me->status = bme68x_get_op_mode(&op_mode, &me->bme6);

	return op_mode;
}

/**
 * @brief Function to get the Temperature, Pressure and Humidity over-sampling
 */
void bme68x_lib_get_tph(bme68x_lib_t * const me, uint8_t * os_hum, uint8_t * os_temp, uint8_t * os_pres) {
	me->status = bme68x_get_conf(&me->conf, &me->bme6);

	if (me->status == BME68X_OK) {
		* os_hum = me->conf.os_hum;
		* os_temp = me->conf.os_temp;
		* os_pres = me->conf.os_pres;
	}
}

/**
 * @brief Function to set the Temperature, Pressure and Humidity over-sampling.
 * Passing no arguments sets the defaults.
 */
void bme68x_lib_set_tph(bme68x_lib_t * const me, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum) {
	me->status = bme68x_get_conf(&me->conf, &me->bme6);

	if (me->status == BME68X_OK) {
		me->conf.os_hum = os_hum;
		me->conf.os_temp = os_temp;
		me->conf.os_pres = os_pres;

		me->status = bme68x_set_conf(&me->conf, &me->bme6);
	}
}

/**
 * @brief Function to get the filter configuration
 */
uint8_t bme68x_lib_get_filter(bme68x_lib_t * const me) {
	me->status = bme68x_get_conf(&me->conf, &me->bme6);

	return me->conf.filter;
}

/**
 * @brief Function to set the filter configuration
 */
void bme68x_lib_set_filter(bme68x_lib_t * const me, uint8_t filter) {
	me->status = bme68x_get_conf(&me->conf, &me->bme6);

	if (me->status == BME68X_OK) {
		me->conf.filter = filter;
		me->status = bme68x_set_conf(&me->conf, &me->bme6);
	}
}

/**
 * @brief Function to get the sleep duration during Sequential mode
 */
uint8_t bme68x_lib_get_seq_sleep(bme68x_lib_t * const me) {
	me->status = bme68x_get_conf(&me->conf, &me->bme6);

	return me->conf.odr;
}

/**
 * @brief Function to set the sleep duration during Sequential mode
 */
void bme68x_lib_set_seq_sleep(bme68x_lib_t * const me, uint8_t odr) {
	me->status = bme68x_get_conf(&me->conf, &me->bme6);

	if (me->status == BME68X_OK) {
		me->conf.odr = odr;
		me->status = bme68x_set_conf(&me->conf, &me->bme6);
	}
}
//
/**
 * @brief Function to set the heater profile for Forced mode
 */
void bme68x_lib_set_heater_prof_for(bme68x_lib_t * const me, uint16_t temp, uint16_t dur) {
	me->heatr_conf.enable = BME68X_ENABLE;
	me->heatr_conf.heatr_temp = temp;
	me->heatr_conf.heatr_dur = dur;

	me->status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &me->heatr_conf, &me->bme6);
}

/**
 * @brief Function to set the heater profile for Sequential mode
 */
void bme68x_lib_set_heater_prof_seq(bme68x_lib_t * const me, uint16_t * temp, uint16_t * dur, uint8_t profile_len) {
	me->heatr_conf.enable = BME68X_ENABLE;
	me->heatr_conf.heatr_temp_prof = temp;
	me->heatr_conf.heatr_dur_prof = dur;
	me->heatr_conf.profile_len = profile_len;

	me->status = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, &me->heatr_conf, &me->bme6);
}

/**
 * @brief Function to set the heater profile for Parallel mode
 */
void bme68x_lib_set_heater_prof_par(bme68x_lib_t * const me, uint16_t * temp, uint16_t * mul, uint16_t shared_heatr_dur, uint8_t profile_len) {
	me->heatr_conf.enable = BME68X_ENABLE;
	me->heatr_conf.heatr_temp_prof = temp;
	me->heatr_conf.heatr_dur_prof = mul;
	me->heatr_conf.shared_heatr_dur = shared_heatr_dur;
	me->heatr_conf.profile_len = profile_len;

	me->status = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &me->heatr_conf, &me->bme6);
}

/**
 * @brief Function to fetch data from the sensor into the local buffer
 */
uint8_t bme68x_lib_fetch_data(bme68x_lib_t * const me) {
	me->n_fields = 0;
	me->status = bme68x_get_data(me->last_op_mode, me->sensor_data, &me->n_fields, &me->bme6);
	me->i_fields = 0;

	return me->n_fields;
}

/**
 * @brief Function to get a single data field
 */
uint8_t bme68x_lib_get_data(bme68x_lib_t * const me, bme68x_data_t * data) {
	if (me->last_op_mode == BME68X_FORCED_MODE) {
		* data = me->sensor_data[0];
	}
	else {
		if (me->n_fields) {
			/* iFields spans from 0-2 while nFields spans from
			 * 0-3, where 0 means that there is no new data
			 */
			* data = me->sensor_data[me->i_fields];
			me->i_fields++;

			/* Limit reading continuously to the last fields read */
			if (me->i_fields >= me->n_fields) {
				me->i_fields = me->n_fields - 1;

				return 0;
			}

			/* Indicate if there is something left to read */
			return me->n_fields - me->i_fields;
		}
	}

	return 0;
}

/**
 * @brief Function to get whole sensor data
 */
bme68x_data_t * bme68x_lib_get_all_data(bme68x_lib_t * const me) {
	return me->sensor_data;
}

/**
* @brief Function to get the BME68x heater configuration
*/
bme68x_heatr_conf_t get_heater_configuration(bme68x_lib_t * const me) {
	return me->heatr_conf;
}

/**
 * @brief Function to retrieve the sensor's unique ID
 */
uint32_t bme68x_lib_get_unique_id(bme68x_lib_t * const me) {
	uint8_t id_regs[4];
	uint32_t uid;

	bme68x_lib_read_multiple_reg(me, BME68X_REG_UNIQUE_ID, id_regs, 4);

	uint32_t id1 = ((uint32_t)id_regs[3] + ((uint32_t)id_regs[2] << 8)) & 0x7FFF;
  uid = (id1 << 16) + (((uint32_t)id_regs[1]) << 8) + (uint32_t)id_regs[0];

  return uid;
}

/**
 * @brief Function to get the error code of the interface functions
 */
BME68X_INTF_RET_TYPE bme68x_lib_intf_error(bme68x_lib_t * const me) {
	return me->bme6.intf_rslt;
}

/**
 * @brief Function to check if an error / warning has occurred
 */
int8_t bme68x_lib_check_status(bme68x_lib_t * const me) {
	if (me->status < BME68X_OK) {
		return BME68X_ERROR;
	}
	else if(me->status > BME68X_OK) {
		return BME68X_WARNING;
	}
	else {
		return BME68X_OK;
	}
}

/* Private functions ---------------------------------------------------------*/
static void delay_us(uint32_t period_us, void * intf_ptr) {
	(void)intf_ptr;
	uint64_t m = (uint64_t)esp_timer_get_time();

  if (period_us) {
  	uint64_t e = (m + period_us);

  	if (m > e) { /* overflow */
  		while ((uint64_t)esp_timer_get_time() > e) {
  			NOP();
  		}
  	}

  	while ((uint64_t)esp_timer_get_time() < e) {
  		NOP();
  	}
  }
}

static int8_t spi_write(uint8_t reg_addr, const uint8_t * reg_data,
		uint32_t length, void * intf_ptr) {
	return BME68X_OK;
}

static int8_t spi_read(uint8_t reg_addr, uint8_t * reg_data, uint32_t length,
		void * intf_ptr) {
	return BME68X_OK;
}

static esp_err_t i2c_init(i2c_t * i2c) {
  i2c_param_config(i2c->i2c_num, &i2c->i2c_conf);

  return i2c_driver_install(i2c->i2c_num, i2c->i2c_conf.mode, 0, 0, 0);
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t * reg_data,
		uint32_t length, void * intf_ptr) {
  uint32_t i;
	esp_err_t ret = ESP_OK;
	int8_t rslt = BME68X_OK;
	bme68x_scomm_t * comm = NULL;

#ifdef BME68X_I2C_BUFFER_SIZE
	if (length + 1 > BME68X_I2C_BUFFER_SIZE) {
		return BME68X_E_COM_FAIL;
	}
#endif

	if (intf_ptr) {
		comm = (bme68x_scomm_t *)intf_ptr;

		/* I2C write */
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    ret = i2c_master_start(handle);
    if (ret != ESP_OK) {
    	i2c_cmd_link_delete_static(handle);
    }

    ret = i2c_master_write_byte(handle, comm->i2c->i2c_addr << 1 | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) {
    	i2c_cmd_link_delete_static(handle);
    }

		ret = i2c_master_write_byte(handle, reg_addr, true);

    for (i = 0; i < length; i++) {
//      uint8_t write_buf[2] = {reg_addr, reg_data[i]};
//			ret = i2c_master_write(handle, write_buf, sizeof(write_buf), true);
    	ret = i2c_master_write_byte(handle, reg_data[i], true);
    }

    i2c_master_stop(handle);
    ret = i2c_master_cmd_begin(comm->i2c->i2c_num, handle, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(handle);

    if (ret != ESP_OK) {
    	rslt = BME68X_E_COM_FAIL;
    }
	}
	else {
		rslt = BME68X_E_NULL_PTR;
	}

	return rslt;
}

static int8_t i2c_read(uint8_t reg_addr, uint8_t * reg_data, uint32_t length,
		void * intf_ptr) {
	esp_err_t ret = ESP_OK;
	int8_t rslt = BME68X_OK;
	bme68x_scomm_t * comm = NULL;

#ifdef BME68X_I2C_BUFFER_SIZE
	if (length > BME68X_I2C_BUFFER_SIZE) {
		return BME68X_E_COM_FAIL;
	}
#endif

	if (intf_ptr) {
		comm = (bme68x_scomm_t *)intf_ptr;

    ret = i2c_master_write_read_device(comm->i2c->i2c_num, comm->i2c->i2c_addr, &reg_addr, 1, reg_data, length, 1000 / portTICK_PERIOD_MS);

    if (ret != ESP_OK) {
    	rslt = BME68X_E_COM_FAIL;
    }
	}
	else {
		rslt = BME68X_E_NULL_PTR;
	}

	return rslt;
}

/***************************** END OF FILE ************************************/
