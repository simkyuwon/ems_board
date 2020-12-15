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
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "ems_server.h"
#include "app_ems_board.h"

/* Logging and RTT */
#include "nrf_log.h"
#include "log.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "ble_softdevice_support.h"

/*  */
#include "ems_pwm.h"
#include "ems_saadc.h"
#include "ems_gpio.h"
#include "ems_board.h"
#include "ems_rtc.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define APP_EMS_PWM_ELEMENT_INDEX   (0)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION      (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                (NRF_MESH_TRANSMIC_SIZE_SMALL)

#define PWM_NORMAL_SEQUENCE_NUMBER  (0)
#define PWM_SIN_SEQUENCE_NUMBER     (1)
#define PWM_TEST_SEQUENCE_NUMBER    (2)

/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_ems_server_set_cb(ems_msg_type_t command, uint8_t position, int32_t data);
static void app_ems_server_get_cb(const app_ems_server_t * p_server, uint8_t * p_position);
static void pad_voltage_control(void);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static board_state ems_board;

static const uint16_t origin_normal_sequence[] = {100};                                                                 //voltage pwm form
static const uint16_t origin_sin_sequence[] = {0, 17, 34, 50, 64, 76, 86, 93, 98, 100, 98, 93, 86, 76, 64, 50, 34, 17}; 
static const uint16_t origin_test_sequence[] = {25, 100};

static const pwm_sequence_config_t m_normal_pwm_sequence_config = PWM_SEQUENCE_CONFIG(origin_normal_sequence, //pwm form
                                                                                      1000);                  //period per step(ms)
static const pwm_sequence_config_t m_sin_pwm_sequence_config = PWM_SEQUENCE_CONFIG(origin_sin_sequence, 500);
static const pwm_sequence_config_t m_test_pwm_sequence_config = PWM_SEQUENCE_CONFIG(origin_test_sequence, 2000);

static bool m_device_provisioned;
static pwm_sequence_config_t m_pwm_sequence_config[] = { m_normal_pwm_sequence_config,
                                                         m_sin_pwm_sequence_config,
                                                         m_test_pwm_sequence_config};

static waveform_pwm_config_t m_waveform_pwm_config =  WAVEFORM_PWM_CONFIG(44000,                //waveform period(us)
                                                                          400,                  //waveform width(us)
                                                                          3,                    //waveform output count
                                                                          PAD_LEFT_PWM_PIN,     //pwm output pin
                                                                          PAD_RIGHT_PWM_PIN);   //pwm output pin
static pad_voltage_pwm_config_t m_voltage_pwm_config = PAD_VOLTAGE_PWM_CONFIG(PAD_VOLTAGE_PWM_PIN,                                //pwm output pin
                                                                              &m_pwm_sequence_config[PWM_SIN_SEQUENCE_NUMBER], //pwm form
                                                                              NRF_TIMER4);                                        //sequence counter
static peltier_pwm_config_t m_peltier_pwm_config = PELTIER_PWM_CONFIG(1000, PELTIER_HEATING_PWM_PIN, PELTIER_COOLING_PWM_PIN);

static saadc_config_t m_pad_voltage_saadc_config = PAD_VOLTAGE_SAADC_CONFIG(NRF_SAADC_RESISTOR_DISABLED,   //resistor bypass
                                                                            NRF_SAADC_GAIN1_6,             //gain 1/6
                                                                            NRF_SAADC_REFERENCE_INTERNAL,  //reference internal(0.6V) 
                                                                            NRF_SAADC_ACQTIME_5US,         //acquisition time 5us, maximum source resistance 50kOhm
                                                                            PAD_VOLTAGE_ANALOG_PIN);       //analog input(P) pin
static saadc_config_t m_peltier_voltage_saadc_config = PELTIER_VOLTAGE_SAADC_CONFIG(NRF_SAADC_RESISTOR_VDD1_2,     //resistor VDD/2
                                                                                    NRF_SAADC_GAIN1_6,             //gain 1/6
                                                                                    NRF_SAADC_REFERENCE_INTERNAL,  //reference internal(0.6V) 
                                                                                    NRF_SAADC_ACQTIME_3US,         //acquisition time 3us, maximum source resistance 10kOhm
                                                                                    PELTIER_VOLTAGE_ANALOG_PIN);   //analog input(P) pin
static saadc_config_t m_temperature_sensor_saadc_config = TEMPERATURE_DIFF_SAADC_CONFIG(NRF_SAADC_RESISTOR_DISABLED,   //V(P) resistor bypass
                                                                                        NRF_SAADC_RESISTOR_DISABLED,   //V(N) resistor bypass
                                                                                        NRF_SAADC_GAIN4,               //gain 4
                                                                                        NRF_SAADC_REFERENCE_INTERNAL,  //reference internal(0.6V)
                                                                                        NRF_SAADC_ACQTIME_3US,         //acquisition time 3us, maximum source resistance 10kOhm
                                                                                        TEMPERATURE_ANALOG_P_PIN,      //analog input(P) pin
                                                                                        TEMPERATURE_ANALOG_N_PIN);     //analog input(N) pin
static saadc_config_t m_temperature_vin_saadc_config = TEMPERATURE_VIN_SAADC_CONFIG(NRF_SAADC_RESISTOR_DISABLED,    //V(P) resistor bypass
                                                                                    NRF_SAADC_GAIN1,                //gain 1
                                                                                    NRF_SAADC_REFERENCE_INTERNAL,   //refernece internal(0.6V)
                                                                                    NRF_SAADC_ACQTIME_3US,          //acquisition time 3us, maximum source resistance 10kOhm
                                                                                    TEMPERATURE_ANALOG_VIN_PIN);    //analog input(P) pin

APP_EMS_PWM_SERVER_DEF(m_ems_server,
                      APP_FORCE_SEGMENTATION,
                      APP_MIC_SIZE,
                      app_ems_server_set_cb,
                      app_ems_server_get_cb)
/*************************************************************************************************/

static void app_ems_server_set_cb(ems_msg_type_t command, uint8_t position, int32_t data)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SET)command : %x,  position : %u, data : %d\n", command, position, data);
    if(position & (1 << ems_board.position))
    {
        if(ems_board.control_mode == BUTTON_CONTROL)
        {
            if(command == CMD_CONTROL_BLE)
            {
                ble_control_mode(&ems_board);
            }
        }
        else if(ems_board.control_mode == BLE_CONTROL)
        {
            switch(command)
            {
                case CMD_CONTROL_BUTTON:
                    button_control_mode(&ems_board);
                    break;
                case CMD_CONTROL_BLE:
                    ble_control_mode(&ems_board);
                    break;

                case CMD_WAVEFORM_START:
                    pwm_start(WAVEFORM_PWM_NUMBER);
                    break;
                case CMD_WAVEFORM_STOP:
                    pwm_stop(WAVEFORM_PWM_NUMBER);
                    break;
                case CMD_WAVEFORM_SINGLESHOT:
                    waveform_single_shot((uint32_t)data);
                    break;
                case CMD_WAVEFORM_PULSE_COUNT_SET:
                    waveform_pulse_count_set(WAVEFORM_PWM_NUMBER, &m_waveform_pwm_config, (uint16_t)data);
                    break;
                case CMD_WAVEFORM_PULSE_PERIOD_SET:
                    waveform_pulse_period_set(WAVEFORM_PWM_NUMBER, &m_waveform_pwm_config, (uint32_t)data);
                    break;
                case CMD_WAVEFORM_PULSE_WIDTH_SET:
                    waveform_pulse_width_set(WAVEFORM_PWM_NUMBER, &m_waveform_pwm_config, (uint16_t)data);
                    break;

                case CMD_VOLTAGE_SEQ_PERIOD_SET:
                    pad_voltage_sequence_period_set(&m_voltage_pwm_config, (uint32_t)data);
                    break;
                case CMD_VOLTAGE_LEVEL_SET:
                    pad_voltage_set(&ems_board, (double)data / 1000.0F);
                    break;
                case CMD_VOLTAGE_SEQUENCE_SET:
                    if((uint32_t)data < ARRAY_SIZE(m_pwm_sequence_config))
                    {
                        pad_voltage_sequence_mode_set(&m_voltage_pwm_config, &m_pwm_sequence_config[(uint32_t)data]);
                    }
                    break;
                case CMD_VOLTAGE_PERIOD_SET:
                    pad_voltage_period_set((uint32_t)data);
                    break;
                case CMD_VOLTAGE_DUTY_SET:
                    pad_voltage_duty_set(&m_voltage_pwm_config, (double)data / 1000.0F);
                    break;
                case CMD_VOLTAGE_COMP_SET:
                    pad_voltage_comp_set(&m_voltage_pwm_config, (uint16_t)data);
                    break;

                case CMD_PELTIER_STOP:
                    peltier_stop();
                    break;
                case CMD_PELTIER_TEMPERATURE_SET:
                    ems_board.peltier_target_temperature = (double)data / 1000.0F;
                    break;
                case CMD_PELTIER_TIMER_SET:
                     if(pwm_state_get(PELTIER_PWM_NUMBER))
                     {
                        rtc2_interrupt((uint32_t)data, (void *)peltier_stop);
                     }
                    break;
                case CMD_PELTIER_HEATING:
                    peltier_heating((uint32_t)data);
                    break;
                case CMD_PELTIER_COOLING:
                    peltier_cooling((uint32_t)data);
                    break;
            }
        }
    }
}

static void app_ems_server_get_cb(const app_ems_server_t * p_server, uint8_t * p_position)
{
    *p_position = ems_board.position;
}

static void app_model_init(void)
{
    ERROR_CHECK(app_ems_init(&m_ems_server, APP_EMS_PWM_ELEMENT_INDEX));
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

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
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

static void ble_mesh_init(void)
{
    ERROR_CHECK(app_timer_init());

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void saadc_init(void)
{
    (void)voltage_saadc_init(&m_pad_voltage_saadc_config.channel_num, &m_pad_voltage_saadc_config);
    (void)voltage_saadc_init(&m_peltier_voltage_saadc_config.channel_num, &m_peltier_voltage_saadc_config);
    (void)temperature_saadc_init(&m_temperature_sensor_saadc_config.channel_num,
                                 &m_temperature_sensor_saadc_config,
                                 &m_temperature_vin_saadc_config.channel_num,
                                 &m_temperature_vin_saadc_config);

    nrf_saadc_init();
}

static void pwm_init(void)
{ 
    waveform_pwm_init(WAVEFORM_PWM_NUMBER, &m_waveform_pwm_config);
    pad_voltage_pwm_init(PAD_VOLTAGE_PWM_NUMBER, &m_voltage_pwm_config);
    peltier_pwm_init(PELTIER_PWM_NUMBER, &m_peltier_pwm_config);
}

static void up_button_callback(void)
{
    pad_voltage_up(&ems_board);
}

static void down_button_callback(void)
{
    pad_voltage_down(&ems_board);
}

static bool mode_button_check(void)
{
    return (gpio_pin_read(MODE_BUTTON) == GPIO_LOW);
}

static void mode_button_callback(void)
{    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "PUSH\n");
    if(rtc2_delay(2000, mode_button_check))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "board off\n");
        board_turn_off();
    }
    else
    {
        uint32_t sequence_size = ARRAY_SIZE(m_pwm_sequence_config);
        uint32_t sequence_index = (m_voltage_pwm_config.p_seq - m_pwm_sequence_config + 1) % sequence_size;
        pad_voltage_sequence_mode_set(&m_voltage_pwm_config, &m_pwm_sequence_config[sequence_index]);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "seq index : %d\n",sequence_index);
    }
}

static void gpio_init(void)
{
    button_event_init(UP_BUTTON,            //gpiote pin
                      up_button_callback);  //event callback function
    button_event_init(DOWN_BUTTON, down_button_callback);
    button_event_init(MODE_BUTTON, mode_button_callback);

    dip_switch_gpio_init(DIP_SWITCH_0);
    dip_switch_gpio_init(DIP_SWITCH_1);
    dip_switch_gpio_init(DIP_SWITCH_2);

    gpio_config_t white_led = LED_GPIO_CONFIG(WHITE_LED, GPIO_PIN_CNF_PULL_Pullup);
    gpio_config_t blue_led  = LED_GPIO_CONFIG(BLUE_LED, GPIO_PIN_CNF_PULL_Pullup);
    gpio_pin_init(&white_led);
    gpio_pin_init(&blue_led);

    gpio_config_t power_control = BOARD_POWER_GPIO_CONFIG(POWER_CONTROL_PIN);
    gpio_pin_init(&power_control);
    gpio_pin_write(POWER_CONTROL_PIN, GPIO_HIGH);
}

static void initialize(void)
{
    nrf_drv_ppi_init();
    nrf_drv_gpiote_init();

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);

    ble_mesh_init();
    gpio_init();
    saadc_init();
    pwm_init();

    rtc2_init();

    ems_board.control_mode                = BLE_CONTROL;
    ems_board.pad_target_voltage          = 30.0F;
    ems_board.peltier_target_temperature  = 25.0F;
}

static void start(void)
{
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
            .p_device_uri = EX_URI_LS_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    pwm_start(WAVEFORM_PWM_NUMBER);
    pwm_start(PAD_VOLTAGE_PWM_NUMBER);
    pwm_start(PELTIER_PWM_NUMBER);
    
    uint32_t dip_state = 0xFF;
    do
    {
        ems_board.position = dip_state;
        read_dip_switch(&dip_state);
    }while(ems_board.position != dip_state);
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }

    return 0;
}

void PWM1_IRQHandler(void)//waveform pwm interrupt
{
    if(NRF_PWM1->EVENTS_PWMPERIODEND)//pad_voltgae_control
    {
        m_voltage_pwm_config.counter->TASKS_CAPTURE[0] = true;
        if(m_voltage_pwm_config.counter->CC[0] % PAD_VOLTAGE_CONTROL_TERM == 0)
        {
            static double prev_voltage = 3.3F;
            double prev_comp = (double)(m_voltage_pwm_config.dma & PWM_COMPARE_Msk);
            double now_voltage = pad_voltage_get(m_pad_voltage_saadc_config.channel_num) * 31;

            //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%dmV\n", (int32_t)(now_voltage * 1000.0F));

            static double prev_target_voltage = -1;
            uint32_t seq_index = (m_voltage_pwm_config.counter->CC[0] * 1000 / PAD_VOLTAGE_HZ / m_voltage_pwm_config.p_seq->period_ms) % m_voltage_pwm_config.p_seq->seq_size;
            double target_voltage = ems_board.pad_target_voltage * (double)m_voltage_pwm_config.p_seq->p_sequence[seq_index] / 100.0F;
            
            double MV = 0.0F;
            const static double Kp = 2.5, Kd = 60, Ki = 0.0005;
            static double integral = 0;
            if(target_voltage != prev_target_voltage)
            {
                prev_target_voltage = target_voltage;
                integral = 0;
            }
            else
            {
                integral += target_voltage - now_voltage;
            }

            MV += (target_voltage - now_voltage) * Kp;
            MV -= (now_voltage - prev_voltage) * Kd;
            MV += integral * Ki;

            prev_comp += MV;

            if(prev_comp > PAD_VOLTAGE_COMP_MAX)
            {
                prev_comp = PAD_VOLTAGE_COMP_MAX;
            }
            if(prev_comp < PAD_VOLTAGE_COMP_MIN)
            {
                prev_comp = PAD_VOLTAGE_COMP_MIN;
            }

            pad_voltage_comp_set(&m_voltage_pwm_config, (uint16_t)prev_comp);

            prev_voltage = now_voltage;
        }
        NRF_PWM1->EVENTS_SEQEND[0] = false;
    }
}

void PWM2_IRQHandler(void)//peltier pwm interrupt
{
    if(NRF_PWM2->EVENTS_SEQEND[0])
    {
        static double prev_temperature = 25.0F;
        double now_temperature = temperature_get(m_temperature_sensor_saadc_config.channel_num, m_temperature_vin_saadc_config.channel_num);

        NRF_PWM2->EVENTS_SEQEND[0] = false;
    }
}
