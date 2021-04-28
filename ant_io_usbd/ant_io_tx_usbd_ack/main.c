/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Garmin Canada Inc. 2015
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 *    1) Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *
 *    2) Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *
 *    3) Neither the name of Garmin nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior
 *       written permission.
 *
 * The following actions are prohibited:
 *
 *    1) Redistribution of source code containing the ANT+ Network
 *       Key. The ANT+ Network Key is available to ANT+ Adopters.
 *       Please refer to http://thisisant.com to become an ANT+
 *       Adopter and access the key. 
 *
 *    2) Reverse engineering, decompilation, and/or disassembly of
 *       software provided in binary form under this license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW 
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 *
 */
/*
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "bsp.h"
#include "boards.h"
#include "hardfault.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_pwr_mgmt.h"
//#include "ant_io_tx.h" // now merged into this file because usb code needed macros to be used in a specific way

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Includes from ant_io_tx.c
#include "string.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_error.h"
#include "sdk_config.h"
#include "ant_channel_config.h"
#include "nrf_soc.h"

// USB defines
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "nrf_drv_clock.h"



// USB DEFINES START  (Initialise the USB CDC_ACM Class)
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

static char m_cdc_data_array[20]; // need to change this

/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// USB DEFINES END



// Other defs

// Other defs

#define LED_FLASH_INTERVAL 1000/(32768/CHAN_PERIOD) // flash for as many ms as an ANT channel period

APP_TIMER_DEF(m_flash_led);


void blink_once_handler(void * p_context)
{
    bsp_board_led_off((uint32_t) p_context);
}

static void flash_led(uint32_t led_idx)
{
    bsp_board_led_on(led_idx);
    ret_code_t err_code = app_timer_start(m_flash_led,
                              APP_TIMER_TICKS(LED_FLASH_INTERVAL),
                              (void *) led_idx);
    APP_ERROR_CHECK(err_code);
}

static void app_timers_setup()
{
    ret_code_t err_code = app_timer_create(&m_flash_led, APP_TIMER_MODE_SINGLE_SHOT, blink_once_handler);
    APP_ERROR_CHECK(err_code);
}





// CODE FROM ant_io_tx.c START
// =============================================================================================================================================================
// Data Page Numbers
#define ANT_CUSTOM_PAGE                 ((uint8_t) 1)           // Page number to identify data format

#define APP_ANT_OBSERVER_PRIO           1                       /**< Application's ANT observer priority. You shouldn't need to modify this value. */
#define FAILED_TX_MESSAGE "TX_FAIL\r\n"

// Static variables and buffers.
static uint8_t m_broadcast_data[ANT_STANDARD_DATA_PAYLOAD_SIZE];    /**< Primary data transmit buffer. */






/**@brief Formats page with current button state and sends data
 * Byte 0   = Page number (Digital I/O Data)
 * Byte 1-6 = Reserved
 * Byte 7   = State of digital inputs
 */
static void handle_transmit()
{
    uint32_t err_code;

    // We only broadcast one format of data, the first byte represents this
    m_broadcast_data[0] = ANT_CUSTOM_PAGE;
    // These rest of the array is set from a serial RX event, until then will be at their initialised values (0 because static global)
    
    err_code = sd_ant_acknowledge_message_tx(ANT_CHANNEL_NUM,
                                           ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                           m_broadcast_data); // This applies to the next message that is sent on the timeslot

    APP_ERROR_CHECK(err_code);
}




void ant_io_tx_setup(void)
{
    uint32_t err_code;

    ant_channel_config_t channel_config =
    {
        .channel_number    = ANT_CHANNEL_NUM,
        .channel_type      = CHANNEL_TYPE_MASTER, // Master acts as the primary transmitter.  Use for sending commands
        .ext_assign        = 0x00,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .device_number     = NRF_FICR->DEVICEID[0], // (FICR = Factory information configuration registers) basically a nordic serial number
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_NETWORK_NUM,
    };

    // Configure channel parameters
    err_code = ant_channel_init(&channel_config);
    APP_ERROR_CHECK(err_code);

    // Open channel.
    err_code = sd_ant_channel_open(ANT_CHANNEL_NUM);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
static void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
    switch (p_ant_evt->event)
    {
        case EVENT_RX:

            //bsp_board_led_invert(BSP_BOARD_LED_2);
            if (p_ant_evt->message.ANT_MESSAGE_aucPayload[0] == ANT_CUSTOM_PAGE) // check rx to see what this is (not done yet)
            {
                //flash_led(BSP_BOARD_LED_2);
                // Set LEDs according to Received Digital IO Data Page
                //m_rx_input_pin_state = p_ant_evt->message.ANT_MESSAGE_aucPayload[7];
                //led_state_set();
            }


            break;

        case EVENT_TX: // Used to initially set the ack_message_tx as default is broadcast
            // MIGHT be worth trying to set the GPIO pin here and see what the logic analyser output looks like
        case EVENT_TRANSFER_TX_COMPLETED: // only triggers if a acknowledged message is successful
            bsp_board_led_invert(BSP_BOARD_LED_2);
            
            handle_transmit();
            break;

        case EVENT_TRANSFER_TX_FAILED:
            bsp_board_led_off(BSP_BOARD_LED_2);
            //flash_led(BSP_BOARD_LED_1);
            
            // Write "failed" to serial the total fail count to serial - rather have the UI keep count than the dongle
            app_usbd_cdc_acm_write(&m_app_cdc_acm, FAILED_TX_MESSAGE, strlen(FAILED_TX_MESSAGE));

            handle_transmit();
        break;

        default:
            break;
    }
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);

// =============================================================================================================================================================
// CODE FROM ant_io_tx.c END







/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    app_timers_setup();

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS,
                        NULL);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 15)); // set gpio 15 to output so we can set and clear it to measure latencies

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for ANT stack initialization.
 */
static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);
}

/**
 *@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}






// USB CODE START
// =============================================================================================================================================================

static bool m_usb_connected = false;

/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: // when the com port is opened - i.e with putty or such
        {
            /*Set up the first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_cdc_data_array,
                                                   1);
            UNUSED_VARIABLE(ret);
            NRF_LOG_INFO("CDC ACM port opened");
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE: // when the com port is closed - i.e with putty or such
            NRF_LOG_INFO("CDC ACM port closed");
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: // happens when data is sent to serial (i.e the user sends via putty)
        {
            ret_code_t ret;
            static uint8_t index = 0;
            uint8_t sent = 0; // track if broadcast has been set
            index++;

            do // this will loop through every character sent
            {
                if ((m_cdc_data_array[index - 1] == '\n') ||
                    (m_cdc_data_array[index - 1] == '\r') ||
                    (index >= 7))
                {
                    if ((index == 7) && (sent == 0)) // When we reach 7 bytes set the broadcast data - anything after is ignored
                    {
                        sent = 1; // to ensure we dont overwrite the data with a message that is too long

                        // THIS IS WHERE I NEED TO SET AND CLEAR THE GPIO PIN, I AM ACTUALLY DUMB
                        // set broadcast data ([0] is set already)
                        m_broadcast_data[1] = m_cdc_data_array[0];
                        m_broadcast_data[2] = m_cdc_data_array[1];
                        m_broadcast_data[3] = m_cdc_data_array[2];
                        m_broadcast_data[4] = m_cdc_data_array[3];
                        m_broadcast_data[5] = m_cdc_data_array[4];
                        m_broadcast_data[6] = m_cdc_data_array[5];
                        m_broadcast_data[7] = m_cdc_data_array[6];
                        nrf_gpio_pin_toggle(NRF_GPIO_PIN_MAP(1, 15));
                    }
                    index = 0;
                }

                // Fetch data until internal buffer is empty, we need to read all of buffer even if we dont use it
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            &m_cdc_data_array[index],
                                            1);
                if (ret == NRF_SUCCESS)
                {
                    index++;
                }
            }
            while (ret == NRF_SUCCESS);
    
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
            break;

        case APP_USBD_EVT_DRV_RESUME:
            break;

        case APP_USBD_EVT_STARTED:
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
        {
            NRF_LOG_INFO("USB power removed");
            m_usb_connected = false;
            app_usbd_stop();
        }
            break;

        case APP_USBD_EVT_POWER_READY:
        {
            NRF_LOG_INFO("USB ready");
            m_usb_connected = true;
            app_usbd_start();
        }
            break;

        default:
            break;
    }
}

// =============================================================================================================================================================
// USB CODE END






/**@brief Function for application main entry. Does not return.
 */
int main(void)
{

    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    log_init();
    utils_setup();

    app_usbd_serial_num_generate();
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    softdevice_setup();
    ant_io_tx_setup();

    NRF_LOG_INFO("ANT IO TX example started.");

    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);

    // Enter main loop.
    for (;;)
    {
        //NRF_LOG_FLUSH();
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        //sd_app_evt_wait(); // wait for an application event
        nrf_pwr_mgmt_run(); // wait for an application event
    }
}
