#include <math.h>
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"
#include "mlx90614.h"

static const uint8_t crc8_table[256]=
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

inline float mlx90614_emissivity_conversion(uint16_t emissivity_val)
{
    return (emissivity_val+1)/65536.0;
}

inline double mlx90614_temp_conversion(uint16_t temperature_val)
{
    return (temperature_val*0.02)-273.15;
}


int mlx90614_crc_cal(mlx90614_reg_data_t * p_raw_data, uint8_t write_read)
{
    uint8_t sum_val=0;

    if(write_read==MLX90614_Read)
    {
        uint8_t pec_array[5]={(MLX90614_ADDR<<1), p_raw_data->reg_addr, ((MLX90614_ADDR<<1)|1),
                            LSB_16(p_raw_data->reg_data), MSB_16(p_raw_data->reg_data)};
        for(int i=0; i<5; i++)
        {
            sum_val=crc8_table[(sum_val^pec_array[i])];
        }        
    }
    else if(write_read==MLX90614_Write)
    {
        uint8_t pec_array[4]={(MLX90614_ADDR<<1), p_raw_data->reg_addr, 
                            LSB_16(p_raw_data->reg_data), MSB_16(p_raw_data->reg_data)};
        for(int i=0; i<4; i++)
        {
            sum_val=crc8_table[(sum_val^pec_array[i])];
        }
    }
    return sum_val;
}

ret_code_t mlx90614_init(mlx90614_instance_t const * p_instance)
{
    ASSERT(p_instance != NULL);

    if (p_instance->p_sensor_data->p_twi_mngr->p_queue->size < MLX90614_MIN_QUEUE_SIZE)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    return NRF_SUCCESS;
}

ret_code_t mlx90614_reg_read(mlx90614_instance_t const * p_instance,
                              mlx90614_reg_data_t *       p_reg_data,
                              mlx90614_data_callback_t    user_cb
                            )
{
    ASSERT(p_instance != NULL);

    return nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                                   p_instance->sensor_addr,
                                   p_reg_data->reg_addr,
                                   (nrf_twi_sensor_reg_cb_t) user_cb,
                                   (uint8_t *) p_reg_data,
                                   MLX90614_REGISTER_BYTES);
}


ret_code_t mlx90614_emissivity_write(mlx90614_instance_t const * p_instance,
                                    float new_emissivity
                                    )
{
    ASSERT(p_instance != NULL);

    if(new_emissivity<0.10 || 1.0<new_emissivity)
    {
        NRF_LOG_WARNING("Invaild Emissivity Value");
        return NRF_ERROR_INVALID_DATA;
    }

    bool idle_chk=false;

    ret_code_t err_code;
    
    uint16_t emissivity_chk_data;
    
    mlx90614_reg_data_t new_emissivity_1;
    new_emissivity_1.reg_addr=MLX90614_REG_EMISSIVITY_1;
    new_emissivity_1.reg_data=(int)(round(65536*(new_emissivity)-1.0));
    new_emissivity_1.pec=mlx90614_crc_cal(&new_emissivity_1,MLX90614_Write);

    emissivity_chk_data=new_emissivity_1.reg_data;

    mlx90614_reg_data_t emissivity_zero;
    emissivity_zero.reg_addr=MLX90614_REG_EMISSIVITY_1;
    emissivity_zero.pec=mlx90614_crc_cal(&emissivity_zero,MLX90614_Write);

    #ifdef MLX90614xCx

    mlx90614_reg_data_t old_emissivity_1;
    old_emissivity_1.reg_addr=MLX90614_REG_EMISSIVITY_1;

    err_code=nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                           p_instance->sensor_addr,
                           MLX90614_REG_EMISSIVITY_1,
                           (nrf_twi_sensor_reg_cb_t) NULL,
                           (uint8_t *) &old_emissivity_1,
                           MLX90614_REGISTER_BYTES);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614xCx : Old emissivity 1 read failed.");
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);

    if(old_emissivity_1.pec!=mlx90614_crc_cal(&old_emissivity_1,MLX90614_Read))
    {
        NRF_LOG_WARNING("MLX90614xCx : Old emissivity 1 CRC doesn't match");
        return NRF_ERROR_INVALID_DATA;
    }
    #endif

    err_code=nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
                               p_instance->sensor_addr,
                               MLX90614_REG_EMISSIVITY_1,
                               (uint8_t *)&emissivity_zero,
                               sizeof(emissivity_zero)/sizeof(uint8_t)
                               );
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614  : Emissivity 1 erase failed");
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);
    nrf_delay_ms(10);

    err_code=nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
                           p_instance->sensor_addr,
                           MLX90614_REG_EMISSIVITY_1,
                           (uint8_t*)&new_emissivity_1,
                           sizeof(new_emissivity_1)/sizeof(uint8_t)
                           );
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614 : New emissivity 1 write failed");
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);
    nrf_delay_ms(10);

    err_code=nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                                   p_instance->sensor_addr,
                                   MLX90614_REG_EMISSIVITY_1,
                                   (nrf_twi_sensor_reg_cb_t) NULL,
                                   (uint8_t *) &new_emissivity_1,
                                   MLX90614_REGISTER_BYTES);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614 : New emissivity 1 read failed", err_code);
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);
    
    if(new_emissivity_1.reg_data!=emissivity_chk_data)
    {
        NRF_LOG_INFO("New Emissivity 1 data doesn't match");
        return NRF_ERROR_INVALID_DATA;
    }

    #ifdef MLX90614xCx

    emissivity_zero.reg_addr=MLX90614_REG_EMISSIVITY_2;
    emissivity_zero.pec=mlx90614_crc_cal(&emissivity_zero, MLX90614_Write);

    mlx90614_reg_data_t new_emissivity_2, old_emissivity_2;

    new_emissivity_2.reg_addr=MLX90614_REG_EMISSIVITY_2;
    old_emissivity_2.reg_addr=MLX90614_REG_EMISSIVITY_2;

    err_code=nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                           p_instance->sensor_addr,
                           MLX90614_REG_EMISSIVITY_2,
                           (nrf_twi_sensor_reg_cb_t) NULL,
                           (uint8_t *) &old_emissivity_2,
                           MLX90614_REGISTER_BYTES);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614xCx : New emissivity 2 read failed");
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);
    if(old_emissivity_2.pec!=mlx90614_crc_cal(&old_emissivity_2,MLX90614_Read))
    {
        NRF_LOG_WARNING("New emissivity 2 CRC doesn't match");
        return NRF_ERROR_INVALID_DATA;
    }

    new_emissivity_2.reg_data=
	(round(((double)old_emissivity_1.reg_data/new_emissivity_1.reg_data)*old_emissivity_2.reg_data));
        
    new_emissivity_2.pec=mlx90614_crc_cal(&new_emissivity_2,MLX90614_Write);
    
    emissivity_chk_data=new_emissivity_2.reg_data;
 
    err_code=nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
                               p_instance->sensor_addr,
                               MLX90614_REG_EMISSIVITY_2,
                               (uint8_t *)&emissivity_zero,
                               sizeof(emissivity_zero)/sizeof(uint8_t)
                               );
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614xCx : Emissivity 2 erase failed", err_code);
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);
    nrf_delay_ms(10);

    err_code=nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
                           p_instance->sensor_addr,
                           MLX90614_REG_EMISSIVITY_2,
                           (uint8_t *)&new_emissivity_2,
                           sizeof(new_emissivity_2)/sizeof(uint8_t)
                           );
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614xCX Write new emissivity 2 fail");
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);
    nrf_delay_ms(10);

    err_code=nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                           p_instance->sensor_addr,
                           MLX90614_REG_EMISSIVITY_2,
                           (nrf_twi_sensor_reg_cb_t) NULL,
                           (uint8_t *) &new_emissivity_2,
                           MLX90614_REGISTER_BYTES);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MLX90614xCx : New emissivity 2 write failed");
        return err_code;
    }
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);
	
    if(new_emissivity_2.reg_data=!emissivity_chk_data)
    {
        NRF_LOG_INFO("New emissivity 2 data doesn't match");
        return NRF_ERROR_INVALID_DATA;
    }
    #endif
    
    return NRF_SUCCESS;
}



ret_code_t mlx90614_sleep_enter(mlx90614_instance_t const * p_instance)
{
    NRF_LOG_INFO("Sleep Mode Enter");
    uint8_t sleep_pec=0xE8;

    return nrf_twi_sensor_reg_write(p_instance->p_sensor_data,
                       p_instance->sensor_addr,
                       MLX90614_REG_SLEEP,
                       (uint8_t*)&sleep_pec,
                       sizeof(sleep_pec)/sizeof(uint8_t)
                       );
}

void mlx90614_sleep_exit(mlx90614_instance_t const * p_instance, uint8_t sda_pin)
{
    NRF_LOG_INFO("Sleep Mode Exit");
    bool idle_chk;
    do
    {
        idle_chk=nrf_twi_mngr_is_idle(p_instance->p_sensor_data->p_twi_mngr);
        nrf_delay_us(10);
    }while(idle_chk!=true);

    nrf_twi_mngr_uninit(p_instance->p_sensor_data->p_twi_mngr);
    nrf_gpio_cfg_output(sda_pin);
    nrf_gpio_pin_write(sda_pin,0);
    nrf_delay_ms(34);
}
