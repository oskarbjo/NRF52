

#include "lsm6dsrx.h"


const char WHO_AM_I = 0b10001111;
static uint8_t m_tx[] = {WHO_AM_I,0x00};
uint8_t       m_rx[sizeof(m_tx)];    /**< RX buffer. */

void readAccelerometer_ext(nrf_drv_spi_t const * const spi,char *output_buffer[6],bool *spi_xfer_done){

    //nrf_drv_spi_transfer(spi, m_tx_buf_all_axes, 6, m_rx_buf_all_axes, 7);
    nrf_drv_spi_transfer(spi, m_tx, 2, m_rx, 2);
    
    //while(!spi_xfer_done){}
    //spi_xfer_done=false;
    //for(int i = 0; i<6; i++){
    //  output_buffer[i] = m_rx_buf_all_axes[i+1];
    //}

}