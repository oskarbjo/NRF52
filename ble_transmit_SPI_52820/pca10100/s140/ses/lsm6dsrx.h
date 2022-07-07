
#ifndef LSM6DSRX_H__
#define LSM6DSRX_H__


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nrf_drv_spi.h"
#include "nrfx_spi.h"

//const char CTRL1_XL = 0b00010000;
//const char CTRL1_XL_SETTINGS = 0b01000000;

#define OUTX_L_A 0b10101000 //First bit is set to 1 to read back register (see lsm6d data sheet on SPI bus interface)
#define OUTX_H_A 0b10101001
#define OUTY_L_A 0b10101010
#define OUTY_H_A 0b10101011
#define OUTZ_L_A 0b10101100
#define OUTZ_H_A 0b10101101

#define OUTX_L_G 0b10100010 //First bit is set to 1 to read back register (see lsm6d data sheet on SPI bus interface)
#define OUTX_H_G 0b10100011
#define OUTY_L_G 0b10100100
#define OUTY_H_G 0b10100101
#define OUTZ_L_G 0b10100110
#define OUTZ_H_G 0b10100111

#define CTRL1_XL 0b00010000
#define CTRL1_XL_SETTINGS 0b01000000 //104Hz

#define CTRL2_G 0b00010001
#define CTRL2_G_SETTINGS 0b01000010 //104Hz


//const char STATUS_REG = 0b10011110;

//static uint8_t       m_tx_buf_all_axes[] = {OUTX_H_A,OUTX_L_A,OUTY_H_A,OUTY_L_A,OUTZ_H_A,OUTZ_L_A,0x00};           /**< TX buffer. */
//static char       m_rx_buf_all_axes[sizeof(m_tx_buf_all_axes)];    /**< RX buffer. */




void readAccelerometer_ext(nrf_drv_spi_t spi,char *output_buffer[6],bool *spi_xfer_done);

char someFunction();


#ifdef __cplusplus
}
#endif

#endif