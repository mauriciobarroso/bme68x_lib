/**
  ******************************************************************************
  * @file           : bme68x_lib.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BME68X_LIB_H_
#define BME68X_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "driver/i2c.h"
#include "bme68x.h"

/* Exported macro ------------------------------------------------------------*/
#define BME68X_ERROR	INT8_C(-1)
#define BME68X_WARNING	INT8_C(1)

/* Exported typedef ----------------------------------------------------------*/
typedef struct bme68x_data				bme68x_data_t;
typedef struct bme68x_dev					bme68x_dev_t;
typedef enum bme68x_intf					bme68x_intf_t;
typedef struct bme68x_conf				bme68x_conf_t;
typedef struct bme68x_heatr_conf	bme68x_heatr_conf_t;

typedef struct {
	uint8_t i2c_addr;
	i2c_config_t i2c_conf;
	int i2c_num;
} i2c_t;

typedef struct {
	uint8_t cs;
} spi_t;

typedef struct {
	i2c_t * i2c;
	spi_t * spi;
} bme68x_scomm_t;

typedef struct {
	int8_t status;
	bme68x_scomm_t comm;
	bme68x_dev_t bme6;
	bme68x_conf_t conf;
	bme68x_heatr_conf_t heatr_conf;
	bme68x_data_t sensor_data[3];
	uint8_t n_fields;
	uint8_t i_fields;
	uint8_t last_op_mode;
} bme68x_lib_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a BME68x instance
 *
 * @param me   : Pointer to a structure instance of bme68x_lib_t
 * @param arg  : Pointer to a structure with the data to initialize the sensor
 *               as a I2C or SPI device
 * @param intf : Type or device, can be I2C or SPI
 *
 * @return Data at that register
 */
esp_err_t bme68x_lib_init(bme68x_lib_t * const me, void * arg, bme68x_intf_t intf);

/**
 * @brief Function to read a register
 *
 * @param me       : Pointer to a structure instance of bme68x_lib_t
 * @param reg_addr : Register address
 *
 * @return Data at that register
 */
uint8_t bme68x_lib_read_single_reg(bme68x_lib_t * const me, uint8_t reg_addr);

/**
 * @brief Function to read multiple registers
 *
 * @param me       : Pointer to a structure instance of bme68x_lib_t
 * @param reg_addr : Start register address
 * @param reg_data : Pointer to store the data
 * @param length   : Number of registers to read
 */
void bme68x_lib_read_multiple_reg(bme68x_lib_t * const me, uint8_t reg_addr, uint8_t * reg_data, uint32_t length);

/**
 * @brief Function to write data to a register
 *
 * @param me       : Pointer to a structure instance of bme68x_lib_t
 * @param reg_addr : Register addresses
 * @param reg_data : Data for that register
 */
void bme68x_lib_write_single_reg(bme68x_lib_t * const me, uint8_t reg_addr, uint8_t reg_data);

/**
 * @brief Function to write multiple registers
 *
 * @param me       : Pointer to a structure instance of bme68x_lib_t
 * @param reg_addr : Pointer to the register addresses
 * @param reg_data : Pointer to the data for those registers
 * @param length   : Number of register to write
 */
void bme68x_lib_write_multiple_reg(bme68x_lib_t * const me, uint8_t * reg_addr, const uint8_t * reg_data, uint32_t length);

/**
 * @brief Function to trigger a soft reset
 *
 * @param me   : Pointer to a structure instance of bme68x_lib_t
 */
void bme68x_lib_soft_reset(bme68x_lib_t * const me);

/**
 * @brief Function to set the ambient temperature for better configuration
 *
 * @param me   : Pointer to a structure instance of bme68x_lib_t
 * @param temp : Temperature in degree Celsius. Default is 25 deg C
 */
void bme68x_lib_set_ambient_temp(bme68x_lib_t * const me, int8_t temp);

/**
 * @brief Function to get the measurement duration in microseconds
 *
 * @param me      : Pointer to a structure instance of bme68x_lib_t
 * @param op_mode : Operation mode of the sensor. Attempts to use the last one if nothing is set
 *
 * @return Temperature, Pressure, Humidity measurement time in microseconds
 */
uint32_t bme68x_lib_get_meas_dur(bme68x_lib_t * const me, uint8_t op_mode);

/**
 * @brief Function to set the operation mode
 *
 * @param me      : Pointer to a structure instance of bme68x_lib_t
 * @param op_mode : BME68X_SLEEP_MODE, BME68X_FORCED_MODE, BME68X_PARALLEL_MODE, BME68X_SEQUENTIAL_MODE
 */
void bme68x_lib_set_op_mode(bme68x_lib_t * const me, uint8_t op_mode);

/**
 * @brief Function to get the operation mode
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 *
 * @return Operation mode : BME68X_SLEEP_MODE, BME68X_FORCED_MODE, BME68X_PARALLEL_MODE, BME68X_SEQUENTIAL_MODE
 */
uint8_t bme68x_lib_get_op_mode(bme68x_lib_t * const me);

/**
 * @brief Function to get the Temperature, Pressure and Humidity over-sampling
 *
 * @param me      : Pointer to a structure instance of bme68x_lib_t
 * @param os_hum  : BME68X_OS_NONE to BME68X_OS_16X
 * @param os_temp : BME68X_OS_NONE to BME68X_OS_16X
 * @param os_pres : BME68X_OS_NONE to BME68X_OS_16X
 */
void bme68x_lib_get_tph(bme68x_lib_t * const me, uint8_t * os_hum, uint8_t * os_temp, uint8_t * os_pres);

/**
 * @brief Function to set the Temperature, Pressure and Humidity over-sampling.
 *        Passing no arguments sets the defaults.
 *
 * @param me      : Pointer to a structure instance of bme68x_lib_t
 * @param os_temp : BME68X_OS_NONE to BME68X_OS_16X
 * @param os_pres : BME68X_OS_NONE to BME68X_OS_16X
 * @param os_hum  : BME68X_OS_NONE to BME68X_OS_16X
 */
void bme68x_lib_set_tph(bme68x_lib_t * const me, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum);

/**
 * @brief Function to get the filter configuration
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 * @return BME68X_FILTER_OFF to BME68X_FILTER_SIZE_127
 */
uint8_t bme68x_lib_get_filter(bme68x_lib_t * const me);

/**
 * @brief Function to set the filter configuration
 *
 * @param me     : Pointer to a structure instance of bme68x_lib_t
 * @param filter : BME68X_FILTER_OFF to BME68X_FILTER_SIZE_127
 */
void bme68x_lib_set_filter(bme68x_lib_t * const me, uint8_t filter);

/**
 * @brief Function to get the sleep duration during Sequential mode
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 *
 * @return BME68X_ODR_NONE to BME68X_ODR_1000_MS
 */
uint8_t bme68x_lib_get_seq_sleep(bme68x_lib_t * const me);

/**
 * @brief Function to set the sleep duration during Sequential mode
 *
 * @param me  : Pointer to a structure instance of bme68x_lib_t
 * @param odr : BME68X_ODR_NONE to BME68X_ODR_1000_MS
 */
void bme68x_lib_set_seq_sleep(bme68x_lib_t * const me, uint8_t odr);

/**
 * @brief Function to set the heater profile for Forced mode
 *
 * @param me   : Pointer to a structure instance of bme68x_lib_t
 * @param temp : Heater temperature in degree Celsius
 * @param dur  : Heating duration in milliseconds
 */
void bme68x_lib_set_heater_prof_for(bme68x_lib_t * const me, uint16_t temp, uint16_t dur);

/**
 * @brief Function to set the heater profile for Sequential mode
 *
 * @param me          : Pointer to a structure instance of bme68x_lib_t
 * @param temp        : Heater temperature profile in degree Celsius
 * @param dur         : Heating duration profile in milliseconds
 * @param profile_len : Length of the profile
 */
void bme68x_lib_set_heater_prof_seq(bme68x_lib_t * const me, uint16_t * temp, uint16_t * dur, uint8_t profile_len);

/**
 * @brief Function to set the heater profile for Parallel mode
 *
 * @param me               : Pointer to a structure instance of bme68x_lib_t
 * @param temp             : Heater temperature profile in degree Celsius
 * @param mul              : Profile of number of repetitions
 * @param shared_heatr_dur : Shared heating duration in milliseconds
 * @param profile_len      : Length of the profile
 */
void bme68x_lib_set_heater_prof_par(bme68x_lib_t * const me, uint16_t * temp, uint16_t * mul, uint16_t shared_heatr_dur, uint8_t profile_len);

/**
 * @brief Function to fetch data from the sensor into the local buffer
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 *
 * @return Number of new data fields
 */
uint8_t bme68x_lib_fetch_data(bme68x_lib_t * const me);

/**
 * @brief Function to get a single data field
 *
 * @param me   : Pointer to a structure instance of bme68x_lib_t
 * @param data : Structure where the data is to be stored
 *
 * @return Number of new fields remaining
 */
uint8_t bme68x_lib_get_data(bme68x_lib_t * const me, bme68x_data_t * data);

/**
 * @brief Function to get whole sensor data
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 *
 * @return Sensor data
 */
bme68x_data_t * bme68x_lib_get_all_data(bme68x_lib_t * const me);

/**
* @brief Function to get the BME68x heater configuration
*
* @param me : Pointer to a structure instance of bme68x_lib_t
*
* @return Gas heater configuration
*/
bme68x_heatr_conf_t get_heater_configuration(bme68x_lib_t * const me);

/**
 * @brief Function to retrieve the sensor's unique ID
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 *
 * @return Unique ID
 */
uint32_t bme68x_lib_get_unique_id(bme68x_lib_t * const me);

/**
 * @brief Function to get the error code of the interface functions
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 *
 * @return Interface return code
 */
BME68X_INTF_RET_TYPE bme68x_lib_intf_error(bme68x_lib_t * const me);

/**
 * @brief Function to check if an error / warning has occurred
 *
 * @param me : Pointer to a structure instance of bme68x_lib_t
 *
 * @return -1 if an error occurred, 1 if warning occured else 0
 */
int8_t bme68x_lib_check_status(bme68x_lib_t * const me);

#ifdef __cplusplus
}
#endif

#endif /* BME68X_LIB_H_ */

/***************************** END OF FILE ************************************/
