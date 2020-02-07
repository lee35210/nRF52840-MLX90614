#ifndef MLX90614_INTERNAL_H
#define MLX90614_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

//Number of bytes to write or read
#define MLX90614_REGISTER_BYTES 3


//MLX90614 RAM, EEPROM REGISTER ADDRESS
#define MLX90614_REG_AMBIENT_TEMP   0x06
#define MLX90614_REG_OBJECT_1_TEMP  0x07
#define MLX90614_REG_OBJECT_2_TEMP  0x08
#define MLX90614_REG_EMISSIVITY_1 0x24
#define MLX90614_REG_EMISSIVITY_2 0x2F
#define MLX90614_REG_SLEEP  0xFF


/**
 * @brief Structure holding sensor instance
 */
typedef struct
{
    nrf_twi_sensor_t * const p_sensor_data;
    uint8_t const            sensor_addr;
} mlx90614_instance_t;


/**
 * @brief Macro that creates sensor instance.
 */
#define MLX90614_INTERNAL_INSTANCE_DEF(_mlx90614_inst_name, _p_twi_sensor, _sensor_address)\
    static mlx90614_instance_t _mlx90614_inst_name =                                     \
    {                                                                                    \
        .p_sensor_data = _p_twi_sensor,                                                  \
        .sensor_addr   = _sensor_address,                                                \
    }


#endif // MLX90614_INTERNAL_H