/**
 * \file
 * \brief 板级配置
 *
 * \internal
 * \par Modification history
 * - 1.00 24-10-14  peace, first implementation
 * \endinternal
 */

/*******************************************************************************
  头文件包含
*******************************************************************************/

#include "board.h"
#include "board_custom.h"
#include "HSLink_Pro_expansion.h"
#include "hpm_adc16_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_usb_drv.h"

#include <stdio.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

/*******************************************************************************
  本地全局变量声明
*******************************************************************************/

/*******************************************************************************
  本地函数声明
*******************************************************************************/

/*******************************************************************************
  本地全局变量定义
*******************************************************************************/

/*******************************************************************************
  本地函数定义
*******************************************************************************/

/**
 * \brief USB 引脚初始化
 */
static void _usb_dp_dm_pins_init (void)
{
    uint8_t tmp;

    /* Disconnect usb dp/dm pins pull down 45ohm resistance */

    while (sysctl_resource_any_is_busy(HPM_SYSCTL)) {}

    if (pllctlv2_xtal_is_stable(HPM_PLLCTLV2) && pllctlv2_xtal_is_enabled(HPM_PLLCTLV2)) {
        if (clock_check_in_group(clock_usb0, 0)) {
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        } else {
            clock_add_to_group(clock_usb0, 0);
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
            clock_remove_from_group(clock_usb0, 0);
        }
    } else {
        tmp = sysctl_resource_target_get_mode(HPM_SYSCTL, sysctl_resource_xtal);
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, 0x03);
        clock_add_to_group(clock_usb0, 0);
        usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        clock_remove_from_group(clock_usb0, 0);
        while (sysctl_resource_target_is_busy(HPM_SYSCTL, sysctl_resource_usb0)) {}
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, tmp);
    }
}

/**
 * \brief 时钟初始化
 */
static void _clk_init (void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);

    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }

    /* group0[0] */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_rom, 0);
    clock_add_to_group(clock_gptmr0, 0);
    clock_add_to_group(clock_gptmr1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_spi1, 0);
    clock_add_to_group(clock_uart0, 0);
    clock_add_to_group(clock_uart3, 0);

    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_watchdog1, 0);
    clock_add_to_group(clock_mbx0, 0);
    clock_add_to_group(clock_tsns, 0);
    clock_add_to_group(clock_crc0, 0);
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_acmp, 0);
    clock_add_to_group(clock_kman, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_usb0, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Bump up DCDC voltage to 1175mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1175);

    /* Configure CPU to 360MHz, AXI/AHB to 120MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 2, 3);

    /* Configure PLL0 Post Divider */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 0, 0); /* PLL0CLK0: 720MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 1, 3); /* PLL0CLK1: 450MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 2, 7); /* PLL0CLK2: 300MHz */

    /* Configure PLL0 Frequency to 720MHz */
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 0, 720000000);

    clock_update_core_clock();

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

/**
 * \brief 打印时钟频率
 */
static void _clk_freq_print (void)
{
    printf("==============================\n");
    printf(" %s clock summary\n", BOARD_NAME);
    printf("==============================\n");
    printf("cpu0:\t\t %luHz\n", clock_get_frequency(clock_cpu0));
    printf("ahb:\t\t %luHz\n", clock_get_frequency(clock_ahb));
    printf("mchtmr0:\t %luHz\n", clock_get_frequency(clock_mchtmr0));
    printf("xpi0:\t\t %luHz\n", clock_get_frequency(clock_xpi0));
    printf("==============================\n");
}

/**
 * \brief 引脚初始化
 */
static void _pin_init (void)
{

    HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_GPIO_B_13;
    HPM_IOC->PAD[IOC_PAD_PB13].PAD_CTL =  IOC_PAD_PAD_CTL_SR_SET(1)|IOC_PAD_PAD_CTL_SPD_SET(3);

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 13, gpiom_core0_fast);
    gpio_set_pin_output(HPM_FGPIO, GPIO_OE_GPIOB, 13);
    gpio_write_pin(HPM_FGPIO, GPIO_DO_GPIOB, 13, 1);

    /* PB14 TRST */
    HPM_IOC->PAD[IOC_PAD_PB14].FUNC_CTL = IOC_PB14_FUNC_CTL_GPIO_B_14;
    gpiom_set_pin_controller(HPM_GPIOM, TRST_GPIO_INDEX, TRST_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TRST_GPIO_CTRL, TRST_GPIO_INDEX, TRST_GPIO_PIN, 1);
    gpio_set_pin_output(TRST_GPIO_CTRL, TRST_GPIO_INDEX, TRST_GPIO_PIN);

    /* PB11 TCK */
    HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL = IOC_PB11_FUNC_CTL_GPIO_B_11;
    gpiom_set_pin_controller(HPM_GPIOM, TCK_GPIO_INDEX, TCK_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TCK_GPIO_CTRL, TCK_GPIO_INDEX, TCK_GPIO_PIN, 0);
    gpio_set_pin_output(TCK_GPIO_CTRL, TCK_GPIO_INDEX, TCK_GPIO_PIN);

    /* PA28 TMS */
    HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_GPIO_A_28;
    gpiom_set_pin_controller(HPM_GPIOM, TMS_GPIO_INDEX, TMS_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TMS_GPIO_CTRL, TMS_GPIO_INDEX, TMS_GPIO_PIN, 0);
    gpio_set_pin_output(TMS_GPIO_CTRL, TMS_GPIO_INDEX, TMS_GPIO_PIN);

    /* PA30 TMS_DIR */
    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_GPIO_A_30;
    gpiom_set_pin_controller(HPM_GPIOM, TMS_DIR_GPIO_INDEX, TMS_DIR_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TMS_DIR_GPIO_CTRL, TMS_DIR_GPIO_INDEX, TMS_DIR_GPIO_PIN, 1);
    gpio_set_pin_output(TMS_DIR_GPIO_CTRL, TMS_DIR_GPIO_INDEX, TMS_DIR_GPIO_PIN);

    /* PA31 SRST_OUT */
    HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_GPIO_B_15;
    gpiom_set_pin_controller(HPM_GPIOM, SRST_OUT_GPIO_INDEX, SRST_OUT_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(SRST_OUT_GPIO_CTRL, SRST_OUT_GPIO_INDEX, SRST_OUT_GPIO_PIN, 0);
    gpio_set_pin_output(SRST_OUT_GPIO_CTRL, SRST_OUT_GPIO_INDEX, SRST_OUT_GPIO_PIN);

    /* PB12 TDO */
    HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_GPIO_B_12;
    gpiom_set_pin_controller(HPM_GPIOM, TDO_GPIO_INDEX, TDO_GPIO_PIN, gpiom_core0_fast);
    gpio_set_pin_input(TDO_GPIO_CTRL, TDO_GPIO_INDEX, TDO_GPIO_PIN);

    /* PB13 TDI */
    HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_GPIO_B_13;
    gpiom_set_pin_controller(HPM_GPIOM, TDI_GPIO_INDEX, TDI_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TDI_GPIO_CTRL, TDI_GPIO_INDEX, TDI_GPIO_PIN, 0);
    gpio_set_pin_output(TDI_GPIO_CTRL, TDI_GPIO_INDEX, TDI_GPIO_PIN);

    /* PY00 LEG Green */
    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = PIOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;
    HPM_IOC->PAD[IOC_PAD_PY00].FUNC_CTL  = IOC_PY00_FUNC_CTL_GPIO_Y_00;
    gpiom_set_pin_controller(HPM_GPIOM, LED_RED_GPIO_INDEX, LED_RED_GPIO_PIN, gpiom_core0_fast);
    gpio_set_pin_output(LED_RED_GPIO_CTRL, LED_RED_GPIO_INDEX, LED_RED_GPIO_PIN);
}

uint32_t current_reload;

static void set_pwm_waveform_edge_aligned_frequency(uint32_t freq) {
  gptmr_channel_config_t config;
  uint32_t gptmr_freq;

  clock_add_to_group(clock_gptmr0, 0);
  gptmr_channel_get_default_config(HPM_GPTMR0, &config);
  gptmr_freq = clock_get_frequency(clock_gptmr0);
  current_reload = gptmr_freq / freq;
  config.reload = current_reload;
  config.cmp_initial_polarity_high = false;
  gptmr_stop_counter(HPM_GPTMR0, 2);
  gptmr_channel_config(HPM_GPTMR0, 2, &config, false);
  gptmr_channel_reset_count(HPM_GPTMR0, 2);
  gptmr_start_counter(HPM_GPTMR0, 2);
}

void init_pwm_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_GPTMR0_COMP_2;
}

static void set_pwm_waveform_edge_aligned_duty(uint8_t duty) {
  uint32_t cmp;
  if (duty > 100) {
    duty = 100;
  }
  cmp = (current_reload * duty) / 100;
  gptmr_update_cmp(HPM_GPTMR0, 2, 0, cmp);
  gptmr_update_cmp(HPM_GPTMR0, 2, 1, current_reload);
}

void board_pwm_init(void) {
  init_pwm_pins();
  set_pwm_waveform_edge_aligned_frequency(500000);
  /* (n/100) *3300 * 100/109.1 *2 */
  /* 54% 对应3.3V 100-54*/
  set_pwm_waveform_edge_aligned_duty(46);
}

/**
 * \brief USB 初始化
 */
static void _usb_init (void)
{
    intc_set_irq_priority(IRQn_USB0, 1);

    usb_hcd_set_power_ctrl_polarity(HPM_USB0, true);
    usb_phy_using_internal_vbus(HPM_USB0);
}

/*******************************************************************************
  外部函数定义
*******************************************************************************/

/**
 * \brief 板级初始化
 */
int32_t board_custom_init (void)
{
    _usb_dp_dm_pins_init();
    _clk_init();
    _clk_freq_print();
    _pin_init();
    // board_pwm_init(); hslink_lite专用
    _usb_init();
    EWDG_Init();// hslinkpro专用
    HSP_Init();// hslinkpro专用

    return 0;
}

/* end of file */