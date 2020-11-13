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
//#include "boards.h"
//#include "pwm_utils.h"
#include "app_timer.h"
//#include "app_pwm.h"

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
#include "app_ems_pwm.h"

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

#define NORMAL_SEQUENCE_NUMBER      (0)
#define SIN_SEQUENCE_NUMBER         (1)
#define TEST_SEQUENCE_NUMBER        (2)

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);

/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_ems_pwm_duty_server_set_cb(const app_ems_pwm_server_t * p_server, int16_t duty);
static void app_ems_pwm_duty_server_get_cb(const app_ems_pwm_server_t * p_server, int16_t * p_present_duty);
static void app_ems_pwm_duty_server_transition_cb(const app_ems_pwm_server_t * p_server,
                                                uint32_t transition_time_ms, int16_t duty);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
 
static const uint16_t origin_normal_sequence[] = {100};                                                                 //voltage pwm form
static const uint16_t origin_sin_sequence[] = {0, 17, 34, 50, 64, 76, 86, 93, 98, 100, 98, 93, 86, 76, 64, 50, 34, 17}; 
static const uint16_t origin_test_sequence[] = {25, 100};

static const pwm_sequence_config_t m_normal_pwm_sequence_config = PWM_SEQUENCE_CONFIG(origin_normal_sequence, //pwm form
                                                                                      1);                     //period per step(ms)
static const pwm_sequence_config_t m_sin_pwm_sequence_config = PWM_SEQUENCE_CONFIG(origin_sin_sequence, 2);
static const pwm_sequence_config_t m_test_pwm_sequence_config = PWM_SEQUENCE_CONFIG(origin_test_sequence, 10);

static bool m_device_provisioned;
static uint16_t saadc_result[SAMPLES_IN_BUFFER * SAADC_CHANNEL_COUNT];
static pwm_sequence_config_t m_pwm_sequence_config[] = { m_normal_pwm_sequence_config,
                                                         m_sin_pwm_sequence_config,
                                                         m_test_pwm_sequence_config};

static waveform_pwm_config_t m_waveform0_pwm_config =  WAVEFORM_PWM_CONFIG(44000,         //waveform period(us)
                                                                           400,           //waveform width(us)
                                                                           3,             //waveform output count
                                                                           PWM_0_L_PIN,   //pwm output pin
                                                                           PWM_0_R_PIN);  //pwm output pin
static waveform_pwm_config_t m_waveform1_pwm_config =  WAVEFORM_PWM_CONFIG(44000, 400, 3, PWM_1_L_PIN, PWM_1_R_PIN);
static voltage_pwm_config_t m_voltage_pwm_config = VOLTAGE_PWM_CONFIG(PWM_VOLTAGE_PIN,                              //pwm output pin
                                                                      &m_pwm_sequence_config[SIN_SEQUENCE_NUMBER]); //pwm form

APP_EMS_PWM_SERVER_DEF(m_ems_pwm_server,
                      APP_FORCE_SEGMENTATION,
                      APP_MIC_SIZE,
                      app_ems_pwm_duty_server_set_cb,
                      app_ems_pwm_duty_server_get_cb,
                      app_ems_pwm_duty_server_transition_cb)

/*************************************************************************************************/

static void app_ems_pwm_duty_server_set_cb(const app_ems_pwm_server_t * p_server, int16_t duty){
    duty -= INT16_MIN;

    //change_voltage(duty * 100 / UINT16_MAX);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "voltage : %d\n", duty * 100 / UINT16_MAX);

    //waveform_pulse_count_change(WAVEFORM0_PWM_NUMBER, &m_waveform0_pwm_config, duty / 1000);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "cnt : %d\n", duty / 1000);
    
    waveform_pulse_period_change(WAVEFORM0_PWM_NUMBER, &m_waveform0_pwm_config, duty + 10000);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "period : %d\n", duty + 10000);
}

static void app_ems_pwm_duty_server_get_cb(const app_ems_pwm_server_t * p_server, int16_t * p_present_duty){
}

static void app_ems_pwm_duty_server_transition_cb(const app_ems_pwm_server_t * p_server,
                                                uint32_t transition_time_ms, int16_t duty){
    waveform_pulse_count_change(WAVEFORM0_PWM_NUMBER, &m_waveform0_pwm_config, duty / 1000);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "cnt : %d\n", duty / 1000);
}

static void app_model_init(void)
{
    ERROR_CHECK(app_ems_pwm_init(&m_ems_pwm_server, APP_EMS_PWM_ELEMENT_INDEX));
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
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
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

static void ble_mesh_init(void){
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
    saadc_config_t m_voltage_saadc_config = VOLTAGE_SAADC_CONFIG(NRF_SAADC_RESISTOR_DISABLED,   //resistor bypass
                                                                 NRF_SAADC_GAIN1_6,             //gain 1/6
                                                                 NRF_SAADC_REFERENCE_INTERNAL,  //reference internal(0.6V) 
                                                                 NRF_SAADC_ACQTIME_3US,         //acquisition time 3us, maximum source resistance 10kOhm
                                                                 VOLTAGE_ANALOG_PIN);           //analog input(P) pin

    saadc_config_t m_temperature_sensor_saadc_config = TEMPERATURE_DIFF_SAADC_CONFIG(NRF_SAADC_RESISTOR_DISABLED,   //V(P) resistor bypass
                                                                                   NRF_SAADC_RESISTOR_DISABLED,   //V(N) resistor bypass
                                                                                   NRF_SAADC_GAIN4,               //gain 4
                                                                                   NRF_SAADC_REFERENCE_INTERNAL,  //reference internal(0.6V)
                                                                                   NRF_SAADC_ACQTIME_3US,         //acquisition time 3us, maximum source resistance 10kOhm
                                                                                   TEMPERATURE_ANALOG_P_PIN,      //analog input(P) pin
                                                                                   TEMPERATURE_ANALOG_N_PIN);     //analog input(N) pin

    saadc_config_t m_temperatur_vss_saadc_config = TEMPERATURE_VSS_SAADC_CONFIG(NRF_SAADC_RESISTOR_DISABLED,    //V(P) resistor bypass
                                                                                NRF_SAADC_GAIN1,                //gain 1
                                                                                NRF_SAADC_REFERENCE_INTERNAL,   //refernece internal(0.6V)
                                                                                NRF_SAADC_ACQTIME_3US,          //acquisition time 3us, maximum source resistance 10kOhm
                                                                                TEMPERATURE_ANALOG_VSS_PIN);    //analog input(P) pin

    if(!nrf_saadc_init(saadc_result, SAMPLES_IN_BUFFER * SAADC_CHANNEL_COUNT))
    {
          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "saadc init failed\n");
    }

    if(!voltage_saadc_init(VOLTAGE_SAADC_CHANNEL, &m_voltage_saadc_config))
    {
          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "voltage saadc init failed\n");
    }

    if(!temperature_saadc_init(TEMPERATURE_SENSOR_SAADC_CHANNEL,
                               &m_temperature_sensor_saadc_config,
                               TEMPERATURE_VSS_SAADC_CHANNEL,
                               &m_temperatur_vss_saadc_config))
    {
          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "temperature saadc init failed\n");
    }
}

static void pwm_init(void)
{
    pwm_sequence_config_t * p_end = m_pwm_sequence_config + sizeof(m_pwm_sequence_config) / sizeof(pwm_sequence_config_t);
    for(pwm_sequence_config_t * p_config = m_pwm_sequence_config; p_config < p_end; p_config++)
    {
        voltage_sequence_init(p_config);
    }

    waveform_pwm_init(WAVEFORM0_PWM_NUMBER, &m_waveform0_pwm_config);
    waveform_pwm_init(WAVEFORM1_PWM_NUMBER, &m_waveform1_pwm_config);
    voltage_pwm_init(VOLTAGE_PWM_NUMBER, &m_voltage_pwm_config);
}

static void voltage_up_callback(void)
{    
    voltage_level_up(m_voltage_pwm_config.p_seq);
}

static void voltage_down_callback(void)
{
    voltage_level_down(m_voltage_pwm_config.p_seq);
}

static void voltage_normal_sequence(void)
{
    //pwm_single_shot(VOLTAGE_PWM_NUMBER);
    //if(m_voltage_pwm_config.p_seq->period_ms > 1)
    //    voltage_period_set(&m_voltage_pwm_config, m_voltage_pwm_config.p_seq->period_ms - 1);
    //voltage_sequence_mode_change(&m_voltage_pwm_config, &m_normal_pwm_sequence_config);
}

static void voltage_sin_sequence(void)
{
    //pwm_start(VOLTAGE_PWM_NUMBER);
    //voltage_period_set(&m_voltage_pwm_config, m_voltage_pwm_config.p_seq->period_ms + 1);
    //voltage_sequence_mode_change(&m_voltage_pwm_config, &m_sin_pwm_sequence_config);
}

static void gpio_init(void)
{
    pulse_generator_init(PULSE_GENERATOR_PIN);

    button_event_init(VOLTAGE_UP_PIN,         //gpiote pin
                      voltage_up_callback);   //event callback function
    button_event_init(VOLTAGE_DOWN_PIN, voltage_down_callback);
    button_event_init(BUTTON_3, voltage_normal_sequence);
    button_event_init(BUTTON_4, voltage_sin_sequence);

    dip_switch_gpio_init(DIP_SWITCH_0);
    dip_switch_gpio_init(DIP_SWITCH_1);
    dip_switch_gpio_init(DIP_SWITCH_2);
}

static void initialize(void)
{
    nrf_drv_ppi_init();
    nrf_drv_gpiote_init();

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    saadc_init();
    ble_mesh_init();
    pwm_init();
    gpio_init();
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

    pwm_start(WAVEFORM0_PWM_NUMBER);
    pwm_start(WAVEFORM1_PWM_NUMBER);
    pwm_start(VOLTAGE_PWM_NUMBER);
}

int main(void)
{
    initialize();
    start();

    uint64_t Vpwm_sum = 0;
    uint64_t Vth_sum = 0;
    uint64_t Vs_sum = 0;
    int saadc_sampling_count = 0;

    for (;;)
    {  
        saadc_buffer_update();
        Vpwm_sum += (uint64_t)saadc_result[0];
        Vth_sum += (uint64_t)saadc_result[1];
        Vs_sum += (uint64_t)saadc_result[2];

        if(++saadc_sampling_count >= 0x8000)
        {
            double Vpwm = Vpwm_sum * 3.6F / saadc_sampling_count / (1<<14);
            double Vs = Vs_sum * 0.6F / saadc_sampling_count / (1<<14);
            double Vth = Vth_sum * 0.6F / 4 / saadc_sampling_count / (1<<13);
            saadc_sampling_count = 0;
            Vpwm_sum = 0;
            Vs_sum = 0;
            Vth_sum = 0;
            double Rth = 220.0F * (Vs - 2 * Vth) / (Vs + 2 * Vth);
            double them = pt100_res2them(Rth);
            //char str[50];
            //int intVs = (int)(Vs * 1000000);
            //int intVth = (int)(Vth * 1000000);
            //int intRth = (int)(Rth * 10000); 
            //int intThem = (int)(them * 10000);
            //sprintf(str, "Vs : %d.%06d Vth : %d.%06d Rth : %d.%04d them:%d.%04dC\n",
            //         intVs/1000000, intVs%1000000, intVth/1000000, intVth%1000000, intRth/10000, intRth%10000, intThem/10000, intThem%10000); 
            //printf("pt100) %s\n", str);

            //int intVpwm = (int)(Vpwm * 1000000);
            //sprintf(str, "Vpwm : %d.%06dV\n", intVpwm/1000000, intVpwm%1000000); 
            //printf("pt100) %s\n", str);
        }
       //(void)sd_app_evt_wait();
    }

    return 0;
}
