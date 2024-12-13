#ifndef _BQ25730_STSTM32
#define _BQ25730_STSTM32

#define FRAMEWORK_ARDUINO   0
#define FRAMEWORK_STM32CUBE 1
#define FRAMEWORK_ZEPHYR    2
#define SELECTED_FRAMEWORK  FRAMEWORK_ARDUINO

#define BQ25730_DEFAULT_ADDR 0x6B
#define ADDR_CHRGOPT0   0x00
#define ADDR_CHRGOPT1   0x30
#define ADDR_CHRGCURR   0x02
#define ADDR_CHRGVOLT   0x04
#define ADDR_ADCPSYS    0x26
#define ADDR_ADCVBUS    0x27
#define ADDR_ADCVBAT    0x2C
#define ADDR_ADCVSYS    0x2D
#define ADDR_VSYSMIN    0x0D
#define ADDR_ADCOPT     0x3A
#define ADDR_ADCIIN     0x2B
#define ADDR_ADCICHG    0x29
#define ADDR_ADCIDCHG   0x28

#define CHRGOPT0_EN_LWPWR   7
#define CHRGOPT0_WDTMR_ADJ  5
#define CHRGOPT1_EN_IBAT    7
#define CHRGOPT1_RSNS_RAC   3
#define CHRGOPT1_RSNS_RSR   2
#define WDTMR_ADJ_DISABLE   0
#define WDTMR_ADJ_5SEC      1
#define WDTMR_ADJ_88SEC     2
#define WDTMR_ADJ_175SEC    3
#define ADCOPT_ADC_CONV     7
#define ADCOPT_ADC_START    6
#define ADC_CONV_ONESHOT    0
#define ADC_CONV_CONT       1
#define RSNS_10MOHM         0
#define RSNS_5MOHM          1

#define VBUS_LSB (float)0.096
#define VSYS_LSB (float)0.064
#define VSYS_OFFSET (float)2.88
#define VBAT_LSB (float)0.064
#define VBAT_OFFSET (float)2.88
#define VSYSMIN_LSB (float)0.1
#define ICHG_10MOHM_LSB (float)0.064
#define ICHG_5MOHM_LSB (float)0.128
#define IDCHG_10MOHM_LSB (float)0.256
#define IDCHG_5MOHM_LSB (float)0.512
#define IIN_10MOHM_LSB (float)0.05
#define IIN_5MOHM_LSB (float)0.1
#define IIN_10MOHM_LSB (float)0.05


#if SELECTED_FRAMEWORK == FRAMEWORK_ARDUINO
#include <Arduino.h>
#include <Wire.h>
#endif

/* 
 * Struct for BQ25730 Configuration 
 * - I2C configuration
 * - GPIO mapping
 * - Chip configuration
 */
typedef struct
{
    uint32_t i2c_freq;
    uint8_t	pin_i2c_sda;
    uint8_t	pin_i2c_scl;
    uint8_t dev_addr;
    uint8_t adc_mode;
    uint8_t watchdog_adj;
    uint8_t rsr;
    uint8_t rac;
    float vsysmin;
    float vcharge;
    float icharge;

}bq25730_config_t;

void i2c_write_registers(uint8_t dev_addr, uint8_t word_addr, uint8_t *data, uint8_t len);
void i2c_read_registers(uint8_t dev_addr, uint8_t word_addr, uint8_t *data, uint8_t len);

void bq25730_init(bq25730_config_t *cfg);
void bq25730_lowpwr_on(bq25730_config_t *cfg);
void bq25730_lowpwr_off(bq25730_config_t *cfg);
void bq25730_set_watchdog(bq25730_config_t *cfg);
void bq25730_adc_enable_all(bq25730_config_t *cfg);
void bq25730_adc_setmode(bq25730_config_t *cfg);
void bq25730_adc_start_conversion(bq25730_config_t *cfg);
float bq25730_read_vbus(bq25730_config_t *cfg);
float bq25730_read_vsys(bq25730_config_t *cfg);
float bq25730_read_vbat(bq25730_config_t *cfg);
float bq25730_read_vsysmin(bq25730_config_t *cfg);
void bq25730_set_vsysmin(bq25730_config_t *cfg);
void bq25730_set_rsense(bq25730_config_t *cfg);
void bq25730_ibat_on(bq25730_config_t *cfg);
void bq25730_ibat_off(bq25730_config_t *cfg);
void bq25730_read_ibat(bq25730_config_t *cfg, float *ibat_charge, float *ibat_discharge);
float bq25730_read_iin(bq25730_config_t *cfg);
void bq25730_set_chargecurrent(bq25730_config_t *cfg);

// Choose to use delay() based on framework
#if SELECTED_FRAMEWORK == FRAMEWORK_ARDUINO
#define delay(x)    delay(x)
#elif SELECTED_FRAMEWORK == FRAMEWORK_STM32CUBE
#define delay(x)    HAL_Delay(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Put non-name-mangling function prototypes in here
 *
 */

#ifdef __cplusplus
}
#endif

#endif
