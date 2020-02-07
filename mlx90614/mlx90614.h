#ifndef MLX90614_H
#define MLX90614_H

#include "nrf_twi_sensor.h"
#include "mlx90614_internal.h"
#ifdef __cplusplus
extern "C" {
#endif

//If you use MLX90614xCx model, Must uncomment this macro.
//#define MLX90614xCx

//Used for calculating CRC-8(indicate transfer direction)
#define MLX90614_Write  0
#define MLX90614_Read   1

/**
 * @brief Possible sensor addresses.
 */
#define MLX90614_ADDR 0x5A


// Minimum nrf_twi_sensor message buffer size and nrf_twi_mngr queue length.
#define MLX90614_MIN_QUEUE_SIZE 4

//TWI PIN
#define MLX90614_SDA_PIN 28
#define MLX90614_SCL_PIN 29


/*
- MLX9061 transfer struct
reg_data : save data reading from mlx90614 or be written in mlx90614 register.
pec : Packet Error Code
reg_addr : MLX90614 Register Address
*/
typedef struct
{
    uint16_t reg_data;
    uint8_t pec;
    uint8_t reg_addr;
} mlx90614_reg_data_t;


/**
 * @brief Data callback prototype.
 *
 * @param[in] result      Result of operation (NRF_SUCCESS on success,
 *                        otherwise a relevant error code).
 * @param[in] p_raw_data  Pointer to raw sensor data structure.
 */
typedef void (* mlx90614_data_callback_t)(ret_code_t result, mlx90614_reg_data_t * p_raw_data);


/**
 * @brief Macro creating mlx90614 sensor instance.
 *
 * @param[in] _mlx90614_inst_name    Sensor instance name.
 * @param[in] _p_twi_sensor         Pointer to common TWI sensor instance.
 * @param[in] _sensor_address       Sensor base address.
 */
#define MLX90614_INSTANCE_DEF(_mlx90614_inst_name, _p_twi_sensor, _sensor_address)                \
    MLX90614_INTERNAL_INSTANCE_DEF(_mlx90614_inst_name, _p_twi_sensor, _sensor_address)


//Emissivity conversion Function
float mlx90614_emissivity_conversion(uint16_t emissivity_val);

//Temperature conversion Function
double mlx90614_temp_conversion(uint16_t emissivity_val);


//MLX90614 initiallization Function
ret_code_t mlx90614_init(mlx90614_instance_t const * p_instance);


//CRC-8 Calculation Fuction
int mlx90614_crc_cal(mlx90614_reg_data_t * p_raw_data, uint8_t write_read);

/**
 * @brief Function for reading regiser data in eeprom and ram.
 *
 * @param[in]  p_instance    Pointer to sensor instance.
 * @param[in]  user_callback Function to be called when data is gathered.
 * @param[out] p_out_data    Pointer to raw data buffer.
 */
ret_code_t mlx90614_reg_read(mlx90614_instance_t const * p_instance,
                             mlx90614_reg_data_t *       p_reg_data,
                             mlx90614_data_callback_t    user_cb
                             );

/**
 * @brief Function for writing new emissivity value to emissivity register in eeprom.
 *
 * @param[in]  p_instance       Pointer to sensor instance.
 * @param[in]  new_emissivity   New Emissivity value to be written in emissivity register.
 */
ret_code_t mlx90614_emissivity_write(mlx90614_instance_t const * p_instance,
                                    float new_emissivity
                                    );

/**
 * @brief Function for entering sleep mode.
 *
 * @param[in]  p_instance   Pointer to sensor instance.
 */
ret_code_t mlx90614_sleep_enter(mlx90614_instance_t const * p_instance);

/**
 * @brief Function for exiting from sleep mode.
 *
 * @param[in]  p_instance   Pointer to sensor instance.
 * @param[in]  sda_pin      TWI SDA PIN
 */
void mlx90614_sleep_exit(mlx90614_instance_t const * p_instance, uint8_t sda_pin);


#endif // MLX90614_H
