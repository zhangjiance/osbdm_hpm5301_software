/*
 * Copyright (c) 2024 HalfSweet
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "hpm_common.h"
#include <hpm_gpiom_drv.h>
#include <hpm_l1c_drv.h>
#include <hpm_romapi.h>
#include <hpm_ewdg_drv.h>
#include "board.h"
#include "hpm_gptmr_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_adc16_drv.h"
#include "HSLink_Pro_expansion.h"

volatile bool VREF_ENABLE = false;

const double ADC_REF = 3.3;

GPTMR_Type *const USER_PWM = HPM_GPTMR0;
const clock_name_t USER_PWM_CLK = clock_gptmr0;
const uint8_t PWM_CHANNEL = 2;

ADC16_Type *const USER_ADC = HPM_ADC0;

const uint32_t DEFAULT_PWM_FREQ = 100000;
const uint8_t DEFAULT_PWM_DUTY = 50;

const uint8_t DEFAULT_ADC_RUN_MODE = adc16_conv_mode_oneshot;
const uint8_t DEFAULT_ADC_CYCLE = 20;

static uint32_t pwm_current_reload;

static const uint32_t CONFIG_P_EN = IOC_PAD_PA31;

static void set_pwm_waveform_edge_aligned_frequency(uint32_t freq) {
    gptmr_channel_config_t config;
    uint32_t gptmr_freq;

    gptmr_channel_get_default_config(USER_PWM, &config);
    gptmr_freq = clock_get_frequency(USER_PWM_CLK);
    pwm_current_reload = gptmr_freq / freq;
    config.reload = pwm_current_reload;
    config.cmp_initial_polarity_high = true;
    gptmr_stop_counter(USER_PWM, PWM_CHANNEL);
    gptmr_channel_config(USER_PWM, PWM_CHANNEL, &config, false);
    gptmr_channel_reset_count(USER_PWM, PWM_CHANNEL);
    gptmr_start_counter(USER_PWM, PWM_CHANNEL);
}

static void set_pwm_waveform_edge_aligned_duty(uint8_t duty) {
    uint32_t cmp;
    if (duty > 100) {
        duty = 100;
    }
    cmp = ((pwm_current_reload * duty) / 100) + 1;
    gptmr_update_cmp(USER_PWM, PWM_CHANNEL, 0, cmp);
}

static void Power_Enable_Init(void) {
    HPM_IOC->PAD[CONFIG_P_EN].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);

    gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(CONFIG_P_EN), GPIO_GET_PIN_INDEX(CONFIG_P_EN),
                             gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO0, GPIO_GET_PORT_INDEX(CONFIG_P_EN), GPIO_GET_PIN_INDEX(CONFIG_P_EN));
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(CONFIG_P_EN), GPIO_GET_PIN_INDEX(CONFIG_P_EN),0);
}

void Power_Turn_On(void) {
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(CONFIG_P_EN), GPIO_GET_PIN_INDEX(CONFIG_P_EN), 1);
}

void Power_Turn_Off(void) {
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(CONFIG_P_EN), GPIO_GET_PIN_INDEX(CONFIG_P_EN), 0);
}

static void Power_PWM_Init(void) {
    // PA10 100k PWM，占空比50%
    HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_GPTMR0_COMP_2;
    set_pwm_waveform_edge_aligned_frequency(DEFAULT_PWM_FREQ);
    set_pwm_waveform_edge_aligned_duty(DEFAULT_PWM_DUTY);
}

static void Power_Set_TVCC_Voltage(double voltage) {
    // voltage = -DAC + (1974/395)
    // DAC = -voltage + (1974/395)

    // PWM DAC需要输出的电压
    double dac = -voltage + (1974.0 / 395.0);
    if (dac < 0) {
        dac = 0;
    } else if (dac > 3.3) {
        dac = 3.3;
    }

    // 计算PWM占空比
    uint8_t duty = (uint8_t) (dac / 3.3 * 100);
    set_pwm_waveform_edge_aligned_duty(duty);
}

static void ADC_Init() {
    /* Configure the ADC clock from AHB (@200MHz by default)*/
    clock_set_adc_source(clock_adc0, clk_adc_src_ahb0);

    adc16_config_t cfg;

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);

    cfg.res = adc16_res_16_bits;
    cfg.conv_mode = DEFAULT_ADC_RUN_MODE;
    cfg.adc_clk_div = adc16_clock_divider_4;
    cfg.sel_sync_ahb = (clk_adc_src_ahb0 == clock_get_source(BOARD_APP_ADC16_CLK_NAME)) ? true : false;

    if (cfg.conv_mode == adc16_conv_mode_sequence ||
        cfg.conv_mode == adc16_conv_mode_preemption) {
        cfg.adc_ahb_en = true;
    }

    /* adc16 initialization */
    if (adc16_init(USER_ADC, &cfg) == status_success) {
        /* enable irq */
        //        intc_m_enable_irq_with_priority(BOARD_APP_ADC16_IRQn, 1);
    }
}

static void ADC_Channel_Init(USER_ADC_CHANNEL_t channel) {
    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.ch = channel;
    ch_cfg.sample_cycle = DEFAULT_ADC_CYCLE;

    adc16_init_channel(USER_ADC, &ch_cfg);

    adc16_set_nonblocking_read(USER_ADC);

#if defined(ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT) && ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT
    /* enable oneshot mode */
    adc16_enable_oneshot_mode(USER_ADC);
#endif
}

static uint16_t Get_ADC_Value(USER_ADC_CHANNEL_t channel) {
    ADC_Channel_Init(channel);

    uint16_t result = 0;

    if (adc16_get_oneshot_result(USER_ADC, channel, &result) == status_success) {
        if (adc16_is_nonblocking_mode(USER_ADC)) {
            adc16_get_oneshot_result(USER_ADC, channel, &result);
        }
    }

    return result;
}

ATTR_ALWAYS_INLINE
static inline void VREF_Init(void) {
    // VREF 的输入是 PB08
    HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

ATTR_ALWAYS_INLINE
static inline void TVCC_Init(void) {
    // TVCC 的输入是 PB09
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

ATTR_ALWAYS_INLINE
static inline double Get_VREF_Voltage(void) {
    return (double) Get_ADC_Value(USER_ADC_VREF_CHANNEL) * ADC_REF / 65535 * 2;
}

ATTR_ALWAYS_INLINE
static double inline Get_TVCC_Voltage(void) {
    return (double) Get_ADC_Value(USER_ADC_TVCC_CHANNEL) * ADC_REF / 65535 * 2;
}

ATTR_ALWAYS_INLINE
static inline void BOOT_Init(void) {
    // PA03
    HPM_IOC->PAD[IOC_PAD_PA03].FUNC_CTL = IOC_PA03_FUNC_CTL_GPIO_A_03;

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOA, 3, gpiom_soc_gpio0);
    gpio_set_pin_input(HPM_GPIO0, GPIO_OE_GPIOA, 3);
    gpio_disable_pin_interrupt(HPM_GPIO0, GPIO_IE_GPIOA, 3);
}

ATTR_ALWAYS_INLINE
static inline void Port_Enable_Init(void) {
    // PA04
    HPM_IOC->PAD[IOC_PAD_PA04].FUNC_CTL = IOC_PA04_FUNC_CTL_GPIO_A_04;

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOA, 4, gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOA, 4);

    gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOA, 4, 0);
}

ATTR_ALWAYS_INLINE
static inline void Port_Turn_Enable(void) {
    gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOA, 4, 1);
}

ATTR_ALWAYS_INLINE
static inline void Port_Turn_Disable(void) {
    gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOA, 4, 0);
}

ATTR_ALWAYS_INLINE
static inline void Power_Trun(bool on) {
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(CONFIG_P_EN), GPIO_GET_PIN_INDEX(CONFIG_P_EN), on);
}

ATTR_ALWAYS_INLINE
static inline void Port_Turn(bool on) {
    gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOA, 4, on);
}


uint32_t running_status = 0; 

extern "C" void board_timer_process(void)
{
    HSP_Loop();
    ewdg_refresh(HPM_EWDG0);
    (void)running_status;

}

extern "C" void hslink_timer_process(void)
{
    HSP_Loop();
    ewdg_refresh(HPM_EWDG0);
}

extern "C" void EWDG_Init() {
  clock_add_to_group(clock_watchdog0, 0);
  ewdg_config_t config;
  ewdg_get_default_config(HPM_EWDG0, &config);

  config.enable_watchdog = true;
  config.int_rst_config.enable_timeout_reset = true;
  config.ctrl_config.use_lowlevel_timeout = false;
  config.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;

  /* Set the EWDG reset timeout to 5 second */
  config.cnt_src_freq = 32768;
  config.ctrl_config.timeout_reset_us = 5 * 1000 * 1000;

  /* Initialize the WDG */
  hpm_stat_t status = ewdg_init(HPM_EWDG0, &config);
  if (status != status_success) {
    printf(" EWDG initialization failed, error_code=%d\n", status);
  }
}


extern "C" void HSP_Init(void) {
    // 初始化电源部分
    Power_Enable_Init();
    Port_Enable_Init();
    Power_PWM_Init();
    // 初始化ADC部分
    ADC_Init();
    VREF_Init();
    TVCC_Init();
}

extern "C" void HSP_Loop(void) {
    // 检测VREF电压
    double vref = Get_VREF_Voltage();

    if (vref > 1.6) {
        Power_Set_TVCC_Voltage(vref);
        Power_Turn_On();
        Port_Turn_Enable();
        VREF_ENABLE = true;
    } else {
        Power_Set_TVCC_Voltage(3.3f); // TVCC恢复默认设置
        VREF_ENABLE = false;
    }
}