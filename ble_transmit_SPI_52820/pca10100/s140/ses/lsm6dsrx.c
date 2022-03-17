

#include "lsm6dsrx.h"


const char WHO_AM_I = 0b10001111;
static uint8_t m_tx_buf11[] = {0x00,0x00};
uint8_t       m_rx_buf11[sizeof(m_tx_buf11)];    /**< RX buffer. */

void readAccelerometer_ext(nrf_drv_spi_t spi,char *output_buffer[6],bool *spi_xfer_done){

    uint8_t regNr = 0;
      m_tx_buf11[0] = OUTX_L_A;
      for(int i = regNr; i<6; i++){
          nrf_drv_spi_transfer(&spi, m_tx_buf11, 2, m_rx_buf11, 2);
          while(!spi_xfer_done){}
          spi_xfer_done=false;
          if(i>1){
            output_buffer[i] = m_rx_buf11[1];
          }
          m_tx_buf11[0]++;
      }

}

char someFunction(){

return 0xEE;

}