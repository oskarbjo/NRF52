/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
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
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#include "amt.h"
#include "counter.h"

#include "sdk_config.h"
#include "nrf.h"
#include "ble.h"
#include "ble_gatt.h"
#include "ble_hci.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "bsp_btn_ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_cli.h"
#include "nrf_cli_rtt.h"
#include "nrf_cli_uart.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "app_util_platform.h"
#include "nrf_delay.h"
#include "boards.h"

#include "nrf_drv_spi.h"
#include "nrfx_spi.h"

#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"





#define DATA_LENGTH_DEFAULT             27                                              /**< The stack default data length. */
#define DATA_LENGTH_MAX                 251                                             /**< The stack maximum data length. */

#define CONN_INTERVAL_DEFAULT           (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))    /**< Default connection interval used at connection establishment by central side. */

#define CONN_INTERVAL_MIN               (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))    /**< Minimum acceptable connection interval, in units of 1.25 ms. */
#define CONN_INTERVAL_MAX               (uint16_t)(MSEC_TO_UNITS(500, UNIT_1_25_MS))    /**< Maximum acceptable connection interval, in units of 1.25 ms. */
#define CONN_SUP_TIMEOUT                (uint16_t)(MSEC_TO_UNITS(4000,  UNIT_10_MS))    /**< Connection supervisory timeout (4 seconds). */
#define SLAVE_LATENCY                   0                                               /**< Slave latency. */

#define SCAN_ADV_LED                    BSP_BOARD_LED_0
#define READY_LED                       BSP_BOARD_LED_1
#define PROGRESS_LED                    BSP_BOARD_LED_2
#define DONE_LED                        BSP_BOARD_LED_3

#define BOARD_TESTER_BUTTON             BSP_BUTTON_2                                    /**< Button to press at the beginning of the test to indicate that this board is connected to the PC and takes input from it via the UART. */
#define BOARD_DUMMY_BUTTON              BSP_BUTTON_3                                    /**< Button to press at the beginning of the test to indicate that this board is standalone (automatic behavior). */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                             /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define APP_BLE_CONN_CFG_TAG            1                                               /**< Tag that refers to the BLE stack configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                               /**< BLE observer priority of the application. There is no need to modify this value. */


#if APP_USBD_CONFIG_EVENT_QUEUE_ENABLE
#include "nrf_atfifo.h"
#include "nrf_atomic.h"
#endif


static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                            /**< Buffer for storing an encoded advertising set. */

ret_code_t ret;
int j = 0;


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


typedef enum
{
    NOT_SELECTED = 0x00,
    BOARD_TESTER,
    BOARD_DUMMY,
} board_role_t;

typedef struct
{
    uint16_t        att_mtu;                    /**< GATT ATT MTU, in bytes. */
    uint16_t        conn_interval;              /**< Connection interval expressed in units of 1.25 ms. */
    ble_gap_phys_t  phys;                       /**< Preferred PHYs. */
    uint8_t         data_len;                   /**< Data length. */
    bool            conn_evt_len_ext_enabled;   /**< Connection event length extension status. */
} test_params_t;


static void scan_start(void);

NRF_BLE_GQ_DEF(m_ble_gatt_queue,                /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_GATT_DEF(m_gatt);                       /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr,NRF_SDH_BLE_TOTAL_LINK_COUNT);                         /**< Context for the Queued Write module.*/
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);       /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                       /**< Scanning Module instance. */




static nrf_ble_amtc_t     m_amtc;
//static nrf_ble_amts_t     m_amts;
NRF_SDH_BLE_OBSERVER(m_amtc_ble_obs, BLE_AMTC_BLE_OBSERVER_PRIO, nrf_ble_amtc_on_ble_evt, &m_amtc);
//NRF_SDH_BLE_OBSERVER(m_amtc_ble_obs1, BLE_AMTC_BLE_OBSERVER_PRIO, nrf_ble_amtc_on_ble_evt, &m_amtc[1]);
//NRF_SDH_BLE_OBSERVER(m_amts_ble_obs, BLE_AMTS_BLE_OBSERVER_PRIO, nrf_ble_amts_on_ble_evt, &m_amts);

NRF_CLI_UART_DEF(cli_uart, 0, 64, 16);
NRF_CLI_RTT_DEF(cli_rtt);
NRF_CLI_DEF(m_cli_uart, "", &cli_uart.transport, '\r', 4);
NRF_CLI_DEF(m_cli_rtt,  "", &cli_rtt.transport,  '\n', 4);




static board_role_t volatile m_board_role  = NOT_SELECTED;

static bool volatile m_run_test;
static bool volatile m_notif_enabled;
static bool volatile m_mtu_exchanged;
static bool volatile m_data_length_updated;
static bool volatile m_phy_updated;
static bool volatile m_conn_interval_configured;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current BLE connection .*/
static uint8_t m_gap_role     = BLE_GAP_ROLE_INVALID;       /**< BLE role for this connection, see @ref BLE_GAP_ROLES */

// Name to use for advertising and connection.
static char const m_target_periph_name[] = DEVICE_NAME;

// Test parameters.
// Settings like ATT MTU size are set only once, on the dummy board.
// Make sure that defaults are sensible.
static test_params_t m_test_params =
{
    .att_mtu                  = NRF_SDH_BLE_GATT_MAX_MTU_SIZE,
    .data_len                 = NRF_SDH_BLE_GAP_DATA_LENGTH,
    .conn_interval            = CONN_INTERVAL_DEFAULT,
    .conn_evt_len_ext_enabled = true,
    // Only symmetric PHYs are supported.
#if defined(S140)
    .phys.tx_phys             = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
    .phys.rx_phys             = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
#else
    .phys.tx_phys             = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS,
    .phys.rx_phys             = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS,
#endif
};


// Connection parameters requested for connection.
static ble_gap_conn_params_t m_conn_param =
{
    .min_conn_interval = CONN_INTERVAL_MIN,   // Minimum connection interval.
    .max_conn_interval = CONN_INTERVAL_MAX,   // Maximum connection interval.
    .slave_latency     = SLAVE_LATENCY,       // Slave latency.
    .conn_sup_timeout  = CONN_SUP_TIMEOUT     // Supervisory timeout.
};


static void test_terminate(void);
void data_len_set(uint8_t value);





















/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE]; //size=NRF_DRV_USBD_EPSIZE
static bool m_send_flag = 0;

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            //bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        //case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        //    bsp_board_led_off(LED_CDC_ACM_OPEN);
        //    break;
        //case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        //    bsp_board_led_invert(LED_CDC_ACM_TX);
        //    break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
            } while (ret == NRF_SUCCESS);

            //bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            //bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            //bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            //bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}





























char const * phy_str(ble_gap_phys_t phys)
{
    static char const * str[] =
    {
        "1 Mbps",
        "2 Mbps",
        "Coded",
        "Unknown"
    };

    switch (phys.tx_phys)
    {
        case BLE_GAP_PHY_1MBPS:
            return str[0];

        case BLE_GAP_PHY_2MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED:
            return str[1];

        case BLE_GAP_PHY_CODED:
            return str[2];

        default:
            return str[3];
    }
}


static void instructions_print(void)
{
    NRF_LOG_INFO("Type 'config' to change the configuration parameters.");
    NRF_LOG_INFO("You can use the Tab key to autocomplete your input.");
    NRF_LOG_INFO("Type 'run' when you are ready to run the test.");
}


/**@brief Function for handling BLE_GAP_EVT_CONNECTED events.
 * Save the connection handle and GAP role, then discover the peer DB.
 */
static void on_ble_gap_evt_connected(ble_gap_evt_t const * p_gap_evt)
{   

    NRF_LOG_INFO("TRYING TO CONNECT");
    ret_code_t err_code;

    m_conn_handle = p_gap_evt->conn_handle;
    m_gap_role    = p_gap_evt->params.connected.role;

    if (m_gap_role == BLE_GAP_ROLE_PERIPH)
    {
        NRF_LOG_INFO("Connected as a peripheral.");
    }
    else if (m_gap_role == BLE_GAP_ROLE_CENTRAL)
    {
        NRF_LOG_INFO("Connected as a central.");
    }
    

    // DISABLING SCANNING MAKES IT IMPOSSIBLE TO CONNECT TO MULTIPLE DEVICES. COMMENT OUT TO ENABLE MULTICONNECTION
    // Stop scanning and advertising.
    nrf_ble_scan_stop();
    //(void) sd_ble_gap_adv_stop(m_adv_handle);

    bsp_board_leds_off();

    // Assign connection handle to available instance of QWR module.
    uint32_t i = 0;
    for (i; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            NRF_LOG_INFO("Assigning m_qwr handle");
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
        NRF_LOG_INFO("m_qwr length: %d",i);
    }
    NRF_LOG_INFO("Discovering GATT database...");
    err_code  = ble_db_discovery_start(&m_ble_db_discovery, p_gap_evt->conn_handle);
    NRF_LOG_DEBUG(err_code);
    APP_ERROR_CHECK(err_code);

    uint32_t    periph_link_cnt = ble_conn_state_central_conn_count();
    //NRF_LOG_INFO("Number of connections: %d",periph_link_cnt);

    if(i<NRF_SDH_BLE_TOTAL_LINK_COUNT-1){scan_start();}



    //if (m_gap_role == BLE_GAP_ROLE_PERIPH)
    //{
    //    NRF_LOG_INFO("Sending PHY Update, %s.", phy_str(m_test_params.phys));

    //    err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle, &m_test_params.phys);
    //    APP_ERROR_CHECK(err_code);
    //}
}


/**@brief Function for handling BLE_GAP_EVT_DISCONNECTED events.
 * Unset the connection handle and terminate the test.
 */
static void on_ble_gap_evt_disconnected(ble_gap_evt_t const * p_gap_evt)
{
    m_conn_handle = BLE_CONN_HANDLE_INVALID;

    NRF_LOG_DEBUG("Disconnected: reason 0x%x.", p_gap_evt->params.disconnected.reason);

    if (m_run_test)
    {
        NRF_LOG_WARNING("GAP disconnection event received while test was running.")
    }

    bsp_board_leds_off();

    test_terminate();
}


/**@brief Function for handling BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t              err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_ble_gap_evt_connected(p_gap_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_ble_gap_evt_disconnected(p_gap_evt);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            m_conn_interval_configured = true;
            NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
                p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
                p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by the peer.
            ble_gap_conn_params_t params;
            params = p_gap_evt->params.conn_param_update_request.conn_params;
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
                p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
                p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        {
            err_code = sd_ble_gatts_sys_attr_set(p_gap_evt->conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT: // Fall through.
        case BLE_GATTS_EVT_TIMEOUT:
        {
            NRF_LOG_DEBUG("GATT timeout, disconnecting.");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE:
        {
            ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

            if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
            {
                // Ignore LL collisions.
                NRF_LOG_DEBUG("LL transaction collision during PHY update.");
                break;
            }

            m_phy_updated = true;

            ble_gap_phys_t phys = {0};
            phys.tx_phys = p_phy_evt->tx_phy;
            phys.rx_phys = p_phy_evt->rx_phy;
            NRF_LOG_INFO("PHY update %s. PHY set to %s.",
                         (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
                         "accepted" : "rejected",
                         phy_str(phys));
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle, &m_test_params.phys);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief AMT server event handler. */
//static void amts_evt_handler(nrf_ble_amts_evt_t evt)
//{
//    ret_code_t err_code;

//    switch (evt.evt_type)
//    {
//        case NRF_BLE_AMTS_EVT_NOTIF_ENABLED:
//        {
//            NRF_LOG_INFO("Notifications enabled.");

//            bsp_board_led_on(READY_LED);
//            m_notif_enabled = true;

//            if (m_board_role != BOARD_TESTER)
//            {
//                return;
//            }

//            if (m_test_params.conn_interval != CONN_INTERVAL_DEFAULT)
//            {
//                NRF_LOG_DEBUG("Updating connection parameters..");
//                m_conn_param.min_conn_interval = m_test_params.conn_interval;
//                m_conn_param.max_conn_interval = m_test_params.conn_interval;
//                err_code = sd_ble_gap_conn_param_update(m_conn_handle, &m_conn_param);

//                if (err_code != NRF_SUCCESS)
//                {
//                    NRF_LOG_ERROR("sd_ble_gap_conn_param_update() failed: 0x%x.", err_code);
//                }
//            }
//            else
//            {
//                m_conn_interval_configured = true;
//            }
//        } break;

//        case NRF_BLE_AMTS_EVT_NOTIF_DISABLED:
//        {
//            NRF_LOG_INFO("Notifications disabled.");
//            bsp_board_led_off(READY_LED);
//        } break;

//        case NRF_BLE_AMTS_EVT_TRANSFER_1KB:
//        {
//            NRF_LOG_INFO("Sent %u KBytes", (evt.bytes_transfered_cnt / 1024));
//            bsp_board_led_invert(PROGRESS_LED);
//        } break;

//        case NRF_BLE_AMTS_EVT_TRANSFER_FINISHED:
//        {
//            counter_stop();

//            bsp_board_led_off(PROGRESS_LED);
//            bsp_board_led_on(DONE_LED);

//            uint32_t time_ms      = counter_get();
//            uint32_t bit_count    = (evt.bytes_transfered_cnt * 8);
//            float throughput_kbps = ((bit_count / (time_ms / 1000.f)) / 1000.f);

//            NRF_LOG_INFO("Done.");
//            NRF_LOG_INFO("=============================");
//            NRF_LOG_INFO("Time: %u.%.2u seconds elapsed.", (time_ms / 1000), (time_ms % 1000));
//            NRF_LOG_INFO("Throughput: " NRF_LOG_FLOAT_MARKER " Kbps.",
//                         NRF_LOG_FLOAT(throughput_kbps));
//            NRF_LOG_INFO("=============================");
//            NRF_LOG_INFO("Sent %u bytes of ATT payload.", evt.bytes_transfered_cnt);
//            NRF_LOG_INFO("Retrieving amount of bytes received from peer...");

//            err_code = nrf_ble_amtc_rcb_read(&m_amtc);
//            if (err_code != NRF_SUCCESS)
//            {
//                NRF_LOG_ERROR("nrf_ble_amtc_rcb_read() failed: 0x%x.", err_code);
//                test_terminate();
//            }
//        } break;
//    }
//}


/**@brief AMT Client event handler.  */
static void amtc_evt_handler(nrf_ble_amtc_t * p_amt_c, nrf_ble_amtc_evt_t * p_evt)
{
    ret_code_t err_code;
    volatile bool doPrint = false;
    //NRF_LOG_INFO("AMTC event triggered, conn_handle: %d",p_evt->conn_handle);
    switch (p_evt->evt_type)
    {
        case NRF_BLE_AMT_C_EVT_DISCOVERY_COMPLETE:
        {
            NRF_LOG_INFO("AMT service discovered at peer.");

            err_code = nrf_ble_amtc_handles_assign(p_amt_c,
                                                   p_evt->conn_handle,
                                                   &p_evt->params.peer_db);
            APP_ERROR_CHECK(err_code);

            // Enable notifications.
            err_code = nrf_ble_amtc_notif_enable(p_amt_c);
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_AMT_C_EVT_NOTIFICATION:
        {
            static uint32_t bytes_cnt  = 0;
            static uint32_t kbytes_cnt = 0;
            //NRF_LOG_INFO("p_evt->params.hvx.notif_len, %d",p_evt->params.hvx.notif_len);

            if (p_evt->params.hvx.bytes_sent == 0)
            {
                bytes_cnt  = 0;
                kbytes_cnt = 0;
            }

            bytes_cnt += p_evt->params.hvx.notif_len;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // My edit: Get the pointer to the received data the length of which is p_evt->params.hvx.notif_len
            uint8_t * p_received_data = p_evt->p_rcv_data;
            char p_received_data2 = p_evt->p_rcv_data;
            int lengthOf = sizeof(p_received_data2);
            //maybe replace this with function like is_part_of_connection_handles(p_gap_evt->conn_handle)
              if(!doPrint){
                    //NRF_LOG_INFO ("Connection handle: %d",p_evt->conn_handle);
                    //NRF_LOG_INFO("Number of connections: %d",ble_conn_state_central_conn_count());
                     //NRF_LOG_INFO("i = %d",p_received_data[0]);
                    //NRF_LOG_INFO("%d, %d, %d, %d, %d, %d",p_received_data[0],p_received_data[1],p_received_data[2],p_received_data[3],p_received_data[4],p_received_data[5]);
                    size_t size = sprintf(m_tx_buffer, "X: %d, %d, %d, %d, %d, %d, %d\r\n",p_received_data[6],p_received_data[0],p_received_data[1],p_received_data[2],p_received_data[3],p_received_data[4],p_received_data[5]);
                    ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
                    j++;
                    if(j==1000){
                      NRF_LOG_INFO("1000");
                      j=0;
                    }
                    doPrint=true;
                }
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            if (bytes_cnt > 1024)
            {
                bsp_board_led_invert(PROGRESS_LED);

                bytes_cnt -= 1024;
                kbytes_cnt++;

                //NRF_LOG_INFO("Received %u kbytes", kbytes_cnt);

                //nrf_ble_amts_rbc_set(&m_amts, p_evt->params.hvx.bytes_rcvd);
            }

            //if (p_evt->params.hvx.bytes_rcvd >= AMT_BYTE_TRANSFER_CNT)
            //{
            //    bsp_board_led_off(PROGRESS_LED);

            //    bytes_cnt  = 0;
            //    kbytes_cnt = 0;

            //    //NRF_LOG_INFO("Transfer complete, received %u bytes of ATT payload.",
            //                 p_evt->params.hvx.bytes_rcvd);

                //nrf_ble_amts_rbc_set(&m_amts, p_evt->params.hvx.bytes_rcvd);
            //}
        } break;

        case NRF_BLE_AMT_C_EVT_RBC_READ_RSP:
        {
            NRF_LOG_INFO("Peer received %u bytes of ATT payload.", (p_evt->params.rcv_bytes_cnt));
            //test_terminate();
        } break;

        default:
            break;
    }
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective service instances.
 *
 * @param[in] p_evt  Pointer to the database discovery event.
 */
static void db_disc_evt_handler(ble_db_discovery_evt_t * p_evt)
{
    nrf_ble_amtc_on_db_disc_evt(&m_amtc, p_evt);
}


/**@brief Function for handling events from the GATT library. */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            m_mtu_exchanged = true;
            NRF_LOG_INFO("ATT MTU exchange completed. MTU set to %u bytes.",
                         p_evt->params.att_mtu_effective);
            
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            m_data_length_updated = true;
            NRF_LOG_INFO("Data length updated to %u bytes.", p_evt->params.data_length);
        } break;
    }

    //nrf_ble_amts_on_gatt_evt(&m_amts, p_evt);
}


/**@brief Function for setting up advertising data. */
static void advertising_data_set(void)
{
    ret_code_t ret;

    ble_gap_adv_params_t const adv_params =
    {
        .properties    =
        {
          .type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED,
        },
        .p_peer_addr   = NULL,
        .filter_policy = BLE_GAP_ADV_FP_ANY,
        .interval      = ADV_INTERVAL,
        .duration      = 0,

        .primary_phy   = BLE_GAP_PHY_1MBPS, // Must be changed to connect in long range. (BLE_GAP_PHY_CODED)
        .secondary_phy = BLE_GAP_PHY_2MBPS,
    };

    ble_advdata_t const adv_data =
    {
        .name_type          = BLE_ADVDATA_FULL_NAME,
        .flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
        .include_appearance = false,
    };

    ret = ble_advdata_encode(&adv_data, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(ret);

    ret = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for starting advertising. */
//static void advertising_start(void)
//{
//    NRF_LOG_INFO("Starting advertising.");

//    bsp_board_led_on(SCAN_ADV_LED);

//    UNUSED_RETURN_VALUE(sd_ble_gap_adv_stop(m_adv_handle));

//    ret_code_t err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t                       err_code;
    ble_gap_evt_adv_report_t const * p_adv = 
                   p_scan_evt->params.filter_match.p_adv_report;
    ble_gap_scan_params_t    const * p_scan_param = 
                   p_scan_evt->p_scan_params;
    //NRF_LOG_INFO("Scan event triggered");
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
             NRF_LOG_INFO("Device \"%s\" found, sending a connection request.",
             (uint32_t) m_target_periph_name);

            // Stop advertising.
            //err_code = sd_ble_gap_adv_stop(m_adv_handle);
            //if (err_code != NRF_ERROR_INVALID_STATE)
            //{
            //    APP_ERROR_CHECK(err_code);
            //}

            // Initiate connection.
            m_conn_param.min_conn_interval = CONN_INTERVAL_DEFAULT;
            m_conn_param.max_conn_interval = CONN_INTERVAL_DEFAULT;

            err_code = sd_ble_gap_connect(&p_adv->peer_addr,
                                          p_scan_param,
                                          &m_conn_param,
                                          APP_BLE_CONN_CFG_TAG);

            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("sd_ble_gap_connect() failed: 0x%x.", err_code);
            }
        } break;

        default:
            break;
     }
}


/**@brief Function for initialization the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_init(&m_scan, NULL, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, 
                                       SCAN_NAME_FILTER, 
                                       m_target_periph_name);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, 
                                           NRF_BLE_SCAN_NAME_FILTER, 
                                           false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting the scanning. */
static void scan_start(void)
{
    NRF_LOG_INFO("Starting scanning.");

    bsp_board_led_on(SCAN_ADV_LED);

    ret_code_t err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for initializing the timer.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for enabling button input.
 */
static void buttons_enable(void)
{
    ret_code_t err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for disabling button input. */
static void buttons_disable(void)
{
    ret_code_t err_code = app_button_disable();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press or release).
 */
static void button_evt_handler(uint8_t pin_no, uint8_t button_action)
{
    switch (pin_no)
    {
        case BOARD_TESTER_BUTTON:
        {
            NRF_LOG_INFO("This board will act as tester.");
            instructions_print();
            m_board_role = BOARD_TESTER;
        } break;

        case BOARD_DUMMY_BUTTON:
        {
            NRF_LOG_INFO("This board will act as responder.");
            m_board_role = BOARD_DUMMY;

            // Set the data length to the maximum if necessary, so that this board
            // accepts whatever value requested by the tester.

            uint8_t dl = 0;
            (void) nrf_ble_gatt_data_length_get(&m_gatt, BLE_CONN_HANDLE_INVALID, &dl);
            if (dl != DATA_LENGTH_MAX)
            {
                data_len_set(DATA_LENGTH_MAX);
            }

            //advertising_start();
            scan_start();
        } break;

        default:
            break;
    }
    buttons_disable();
}


/**@brief Function for initializing the button library.
 */
static void buttons_init(void)
{
   // The array must be static because a pointer to it will be saved in the button library.
    static app_button_cfg_t buttons[] =
    {
        {BOARD_TESTER_BUTTON, false, BUTTON_PULL, button_evt_handler},
        {BOARD_DUMMY_BUTTON,  false, BUTTON_PULL, button_evt_handler}
    };

    ret_code_t err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


static void client_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_evt_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    /////
    nrf_ble_amtc_init_t amtc_init;
    memset(&amtc_init, 0, sizeof(amtc_init));
    amtc_init.evt_handler  = amtc_evt_handler;
    amtc_init.p_gatt_queue = &m_ble_gatt_queue;

    err_code = nrf_ble_amtc_init(&m_amtc, &amtc_init);
    APP_ERROR_CHECK(err_code);






}






/**@brief Function for handling Queued Write module errors.
 *
 * @details A pointer to this function is passed to each service that may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code that contains information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module instances.
    qwr_init.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_ERROR(err_code);
        NRF_LOG_INFO("Init of m_qwr %d",i);
    }
}





/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing GAP parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device, including the device name and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (uint8_t const *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_ppcp_set(&m_conn_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT library. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t ret;
    ret = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret);
}


void preferred_phy_set(ble_gap_phys_t * p_phy)
{
    memcpy(&m_test_params.phys, p_phy, sizeof(ble_gap_phys_t));
}


void gatt_mtu_set(uint16_t att_mtu)
{
    ret_code_t err_code;

    m_test_params.att_mtu = att_mtu;

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, att_mtu);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, att_mtu);
    APP_ERROR_CHECK(err_code);
}


void connection_interval_set(uint16_t value)
{
    m_test_params.conn_interval = value;
}


void conn_evt_len_ext_set(bool status)
{
    ret_code_t err_code;
    ble_opt_t  opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);

    m_test_params.conn_evt_len_ext_enabled = status;
}


void data_len_set(uint8_t value)
{
    ret_code_t err_code;
    err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, value);
    APP_ERROR_CHECK(err_code);

    m_test_params.data_len = value;
}


bool is_tester_board(void)
{
    return (m_board_role == BOARD_TESTER);
}


void current_config_print(nrf_cli_t const * p_cli)
{
    char const * role = (m_board_role == BOARD_TESTER) ? "tester" :
                        (m_board_role == BOARD_DUMMY)  ? "dummy" : "not selected";

    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "==== Current test configuration ====\r\n");
    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL,
                    "Board role:\t\t%s\r\n"
                    "ATT MTU size:\t\t%d\r\n"
                    "Data length:\t\t%d\r\n"
                    "Connection interval:\t%d units\r\n"
                    "Connection length ext:\t%s\r\n"
                    "Preferred PHY:\t\t%s\r\n",
                    role,
                    m_test_params.att_mtu,
                    m_test_params.data_len,
                    m_test_params.conn_interval,
                    m_test_params.conn_evt_len_ext_enabled ? "on" : "off",
                    phy_str(m_test_params.phys));
    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "GAP event length:\t%d\r\n",
                    NRF_SDH_BLE_GAP_EVENT_LENGTH);
}


void test_begin(void)
{
    NRF_LOG_INFO("Preparing the test.");
    NRF_LOG_FLUSH();

#if defined(S132)
    // PHY does not need to be updated for s132.
     m_phy_updated = true;
#endif

    switch (m_gap_role)
    {
        default:
            // If no connection was established, the role is not established either.
            // In this case, start both the advertising and the scanning.
            //advertising_start();
            scan_start();
            break;

        case BLE_GAP_ROLE_PERIPH:
            //advertising_start();
            m_test_params.phys.tx_phys = BLE_GAP_PHY_2MBPS;
            break;

        case BLE_GAP_ROLE_CENTRAL:
            scan_start();
            break;
    }
}


static void test_run(void)
{
    counter_start();
    //nrf_ble_amts_notif_spam(&m_amts);
}


static bool is_test_ready()
{
    return (   (m_board_role == BOARD_TESTER)
            && m_conn_interval_configured
            && m_notif_enabled
            && m_mtu_exchanged
            && (m_data_length_updated || m_test_params.data_len == DATA_LENGTH_DEFAULT)
            && !m_run_test);
}


void cli_process(void)
{
    nrf_cli_process(&m_cli_uart);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    cli_process();

    //if (is_test_ready())
    //{
    //    NRF_LOG_INFO("Test started");
    //    m_run_test = true;
    //    test_run();
    //}

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


static void test_terminate(void)
{
    m_run_test                 = false;
    m_notif_enabled            = false;
    m_mtu_exchanged            = false;
    m_data_length_updated      = false;
    m_phy_updated              = false;
    m_conn_interval_configured = false;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("Disconnecting...");

        ret_code_t err_code;
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("sd_ble_gap_disconnect() failed: 0x%0x.", err_code);
        }
    }
    else
    {
        if (m_board_role == BOARD_DUMMY)
        {
            if (m_gap_role == BLE_GAP_ROLE_PERIPH)
            {
                //advertising_start();
            }
            else
            {
                scan_start();
            }
        }
    }
}


void cli_init(void)
{
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    {
        ret_code_t err_code = nrf_cli_init(&m_cli_rtt, NULL, true, true, NRF_LOG_SEVERITY_INFO);
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;

    ret_code_t err_code = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(err_code);
}

void cli_start(void)
{

    ret_code_t err_code =  nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(err_code);
}







int main(void)
{
    // Initialize.
    
    log_init();
    //////////  Virtual UART/USB stuff setup: ////////////
    NRF_LOG_INFO("Setting up USBD CDC ACM.");
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }
    ret = app_timer_init();
    APP_ERROR_CHECK(ret);
    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD CDC ACM set up.");

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }
    ////////// END VIRTUAL UART SETUP //////////////////


    
    cli_init();
    leds_init();
    //timer_init(); //already initialized to run virtual USB
    counter_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();

    advertising_data_set();
    scan_init();

    qwr_init();
    client_init();


    gatt_mtu_set(m_test_params.att_mtu);
    conn_evt_len_ext_set(m_test_params.conn_evt_len_ext_enabled);


    

    //// Start execution.
    cli_start();
    buttons_enable();



    NRF_LOG_INFO("ATT MTU example started.");
    NRF_LOG_INFO("Press button 3 on the board connected to the PC.");
    NRF_LOG_INFO("Press button 4 on other board.");
    uint32_t volatile delay_us = 1000000;
    int i = 0;
    m_run_test = false;


    // Enter main loop.
    while(true){
      
      idle_state_handle();
      while (app_usbd_event_queue_process()){}

      #if NRF_CLI_ENABLED
              nrf_cli_process(&m_cli_uart);
      #endif
      //UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
      ///* Sleep CPU only if there was no interrupt since last loop processing */
      //__WFE();
        }
    }


/**
 * @}
 */
