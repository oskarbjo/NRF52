#ifndef LSM6DSRX_H_   /* Include guard */
#define LSM6DSRX_H_

include <stdint.h>

const char CTRL1_XL = 0b00010000;
const char CTRL1_XL_SETTINGS = 0b01000000;

const char CTRL2_G = 0b00010000;
const char CTRL2_G_SETTINGS = 0b01000000;

//#define TEST_STRING "1"
//char TEST_STRING[] = {0x00,0x01};
static uint8_t       m_tx_buf[] = {0b10001111,0x00000000};           /**< TX buffer. */
static uint8_t       m_tx_buf1[] = {CTRL1_XL,CTRL1_XL_SETTINGS};
static uint8_t       m_tx_buf2[] = {0b10101101,0x00};
static uint8_t       m_tx_buf3[] = {0b10101100,0x00};
static uint8_t       m_rx_buf[sizeof(m_tx_buf2)];    /**< RX buffer. */
static uint8_t       m_rx_buf2[sizeof(m_tx_buf2)];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf2)+1;        /**< Transfer length. */