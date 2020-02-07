/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include "boards.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_twi_mngr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_twi_sensor.h"
#include "nrf_delay.h"

#include "mlx90614.h"

#include <stdio.h>

#define TWI_INSTANCE_ID_0 0
#define TWI_INSTANCE_ID_1 1

#define MAX_PENDING_TRANSACTIONS 5

//Macro that simplifies defining a TWI transaction manager instance.
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr_1, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID_1);

//Macro creating common twi sensor instance.
NRF_TWI_SENSOR_DEF(sensor_instance_1, &m_nrf_twi_mngr_1, MAX_PENDING_TRANSACTIONS);

//Macro that creates sensor instance.
MLX90614_INSTANCE_DEF(mlx90614_instance, &sensor_instance_1, MLX90614_ADDR);

APP_TIMER_DEF(m_timer);

mlx90614_reg_data_t emissivity_1_struct;
mlx90614_reg_data_t obj_1_temp_struct;
mlx90614_reg_data_t obj_2_temp_struct;

void log_init(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void twi_config_1(void)
{
    uint32_t err_code;
    
    nrf_drv_twi_config_t const config={
      .scl=MLX90614_SCL_PIN,
      .sda=MLX90614_SDA_PIN,
      .frequency=NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority=APP_IRQ_PRIORITY_MID,
      };
    
    err_code=nrf_twi_mngr_init(&m_nrf_twi_mngr_1,&config);
    APP_ERROR_CHECK(err_code);
}

void mlx90614_read_temp_cb(ret_code_t result, mlx90614_reg_data_t * p_raw_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Read Temperature Callback Error : %d", (int)result);
        return;
    }

    uint8_t crc_value;

    if(p_raw_data->pec!=mlx90614_crc_cal(p_raw_data,MLX90614_Read))
    {
        NRF_LOG_WARNING("MLX90614 Read temperature CRC doesn't match");
        return;
    }

    double temperature;
    temperature=mlx90614_temp_conversion(p_raw_data->reg_data);
    
    NRF_LOG_INFO("Temperautre : " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(temperature));
}


void mlx90614_read_emissivity_cb(ret_code_t result, mlx90614_reg_data_t * p_raw_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Read Emissivity Callback Error : %d", (int)result);
        return;
    }

    uint8_t crc_value;

    if(p_raw_data->pec!=mlx90614_crc_cal(p_raw_data,MLX90614_Read))
    {
        NRF_LOG_WARNING("MLX90614 Read emissivity CRC doesn't match");
        return;
    }

    float emissivity;
    emissivity=mlx90614_emissivity_conversion(p_raw_data->reg_data);
    NRF_LOG_INFO("Current Emissivity : " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(emissivity));
}


static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

void timer_handler(void * p_context)
{
    ret_code_t err_code;
    err_code=mlx90614_reg_read(&mlx90614_instance,&obj_1_temp_struct,mlx90614_read_temp_cb);
    APP_ERROR_CHECK(err_code);
}

void read_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    ret_code_t err_code;

    log_init();
    bsp_board_init(BSP_INIT_LEDS);

    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).
    lfclk_config();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_RAW_INFO("\r\nTWI master example started. \r\n");
    NRF_LOG_FLUSH();
    
    twi_config_1();

    err_code=nrf_twi_sensor_init(&sensor_instance_1);
    APP_ERROR_CHECK(err_code);

    
    emissivity_1_struct.reg_addr=MLX90614_REG_EMISSIVITY_1;
    obj_1_temp_struct.reg_addr=MLX90614_REG_OBJECT_1_TEMP;
    obj_2_temp_struct.reg_addr=MLX90614_REG_OBJECT_2_TEMP;

    err_code=mlx90614_init(&mlx90614_instance);
    APP_ERROR_CHECK(err_code);
    
    //Read current emissivity
    nrf_delay_ms(250);
    err_code=mlx90614_reg_read(&mlx90614_instance,&emissivity_1_struct,mlx90614_read_emissivity_cb);
    APP_ERROR_CHECK(err_code);
    

    /*
        Write new emissivity
    */
//    mlx90614_emissivity_write(&mlx90614_instance,0.9);
//    nrf_delay_ms(250);


    /*
        Enter sleep mode & Exit sleep mode
    */
//    err_code=mlx90614_sleep_enter(&mlx90614_instance);
//    APP_ERROR_CHECK(err_code);
//    nrf_delay_ms(100);
//    mlx90614_sleep_exit(&mlx90614_instance,MLX90614_SDA_PIN);
//    twi_config_1();
//    nrf_delay_ms(250);

    read_init();

    while (true)
    {
        NRF_LOG_FLUSH();
    }
}

/** @} */
