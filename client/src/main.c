/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
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
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "ems_pwm_client.h"
#include "app_ems_pwm.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "example_common.h"
#include "ble_softdevice_support.h"

/* USBD CDC ACM */
#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_drv_power.h"
#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "app_scheduler.h"
/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define APP_UNACK_MSG_REPEAT_COUNT   (2)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION       (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                 (NRF_MESH_TRANSMIC_SIZE_SMALL)

#define MAX_AVAILABLE_SERVER_NODE_NUMBER  (40)
#define CLIENT_MODEL_INSTANCE_COUNT  (1)
#define APP_EMS_PWM_DELAY_MS           (50)
#define APP_EMS_PWM_TRANSITION_TIME_MS (100)

//USB CDC ACM
#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

#define CDC_ACM_BUFFER_SIZE 1024
#define READ_SIZE 1

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);
/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);
/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_ems_pwm_client_status_cb(const ems_pwm_client_t *p_self,
                                         const access_message_rx_meta_t *p_meta,
                                         const ems_pwm_status_params_t *p_in);
static void app_ems_pwm_client_publish_interval_cb(access_model_handle_t handle, void *p_self);
static void app_ems_pwm_client_transaction_status_cb(access_model_handle_t model_handle,
                                                    void *p_args,
                                                    access_reliable_status_t status);

//USB CDC ACM
static void usbd_user_ev_handler(app_usbd_event_type_t event);
/*****************************************************************************
 * Static variables
 *****************************************************************************/
static ems_pwm_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool m_device_provisioned;
//USB CDC ACM
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;
static char m_cdc_tx_buffer[CDC_ACM_BUFFER_SIZE];
static char m_cdc_rx_buffer[CDC_ACM_BUFFER_SIZE];
static bool m_usb_connected = false;

static const ems_pwm_client_callbacks_t client_cbs =
{
    .ems_pwm_status_cb = app_ems_pwm_client_status_cb,
    .ack_transaction_status_cb = app_ems_pwm_client_transaction_status_cb,
    .periodic_publish_cb = app_ems_pwm_client_publish_interval_cb
};

static void app_ems_pwm_client_status_cb(const ems_pwm_client_t *p_self,
                                         const access_message_rx_meta_t *p_meta,
                                         const ems_pwm_status_params_t *p_in)
{
}

static void app_ems_pwm_client_publish_interval_cb(access_model_handle_t handle, void *p_self)
{
}

static void app_ems_pwm_client_transaction_status_cb(access_model_handle_t model_handle,
                                                    void *p_args,
                                                    access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
}

static void provisioning_aborted_cb(void)
{
}

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x\n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t------------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) Send a message to the odd group (address: 0xC003) to turn on LED 1.\n"
    "\t\t Button/RTT 2) Send a message to the odd group (address: 0xC003) to turn off LED 1.\n"
    "\t\t Button/RTT 3) Send a message to the even group (address: 0xC002) to turn on LED 1.\n"
    "\t\t Button/RTT 4) Send a message to the even group (address: 0xC002) to turn off LED 1.\n"
    "\t\t------------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    // Increase button number because the buttons on the board is marked with 1 to 4 
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    ems_pwm_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    switch(button_number)
    {
        case 1:
        case 3:
            set_params.pwm_duty = INT16_MIN;
            break;

        case 2:
        case 4:
            set_params.pwm_duty = INT16_MAX;
            break;
    }

    set_params.tid = tid++;
    transition_params.delay_ms = 0;
    transition_params.transition_time_ms = 0;

    switch (button_number)
    {
        case 1:
        case 2:
            // Demonstrate acknowledged transaction, using 1st client model instance
            // In this examples, users will not be blocked if the model is busy
            (void)access_model_reliable_cancel(m_clients[0].model_handle);
            status = ems_pwm_client_set(&m_clients[0], &set_params, &transition_params);
            break;

        case 3:
        case 4:
            // Demonstrate un-acknowledged transaction, using 2nd client model instance 
            status = ems_pwm_client_set_unack(&m_clients[0], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            break;
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);*/
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;
       
        ERROR_CHECK(ems_pwm_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());

    //usbd
    app_usbd_serial_num_generate();
    APP_ERROR_CHECK(nrf_drv_clock_init());
    
    APP_ERROR_CHECK(app_usbd_init(&usbd_config));

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    APP_ERROR_CHECK(app_usbd_class_append(class_cdc_acm));
#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

//usbd function
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    ret_code_t ret;

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Set up the first transfer*/
            ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                   m_cdc_rx_buffer,
                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
           __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "CDC ACM port opened\n");
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "CDC ACM port closed\n");
            if (m_usb_connected)
            {
            }
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            {
                uint32_t sum = 0;
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Bytes waiting: %d\n", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
                do
                {
                    /*Get amount of data transfered*/
                    size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RX: size: %u char: %c\n", size, m_cdc_rx_buffer[0]);

                   if(m_cdc_rx_buffer[0] >= '0' && m_cdc_rx_buffer[0] <= '9')
                   {
                      sum += sum * 10 + m_cdc_rx_buffer[0] - '0';
                   }
                   else
                   {
                      sum = 0;
                   }

                
                    size = sprintf(m_cdc_tx_buffer, "%c\n\r", m_cdc_rx_buffer[0]);
                    app_usbd_cdc_acm_write(p_cdc_acm,  m_cdc_tx_buffer, size);

                    /* Fetch data until internal buffer is empty */
                    ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                m_cdc_rx_buffer,
                                                READ_SIZE);
                } while (ret == NRF_SUCCESS);

                app_usbd_cdc_acm_write(p_cdc_acm,  m_cdc_tx_buffer, sprintf(m_cdc_tx_buffer, "%d\n\r", sum + 1));
                button_event_handler(sum);
            }
            break;
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
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "USB power detected\n");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
        {
           __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "USB power removed\n");
            m_usb_connected = false;
            app_usbd_stop();
        }
            break;

        case APP_USBD_EVT_POWER_READY:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "USB ready\n");
            m_usb_connected = true;
            app_usbd_start();
        }
            break;

        default:
            break;
    }
}


static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_LS_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());
    APP_ERROR_CHECK(app_usbd_power_events_enable());
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    power_manage();

}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        idle_state_handle();

    }
}
