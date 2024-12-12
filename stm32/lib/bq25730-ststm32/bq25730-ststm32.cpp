#include "bq25730-ststm32.h"



void i2c_write_registers(uint8_t dev_addr, uint8_t word_addr, uint8_t *data, uint8_t len) {
#if SELECTED_FRAMEWORK == FRAMEWORK_ARDUINO
    Wire.begin();
    Wire.beginTransmission(dev_addr);
    Wire.write(word_addr);
    for (int i=0; i<len; i++){
        Wire.write(*(data+i));
    }
    Wire.endTransmission();
#endif
}

void i2c_read_registers(uint8_t dev_addr, uint8_t word_addr, uint8_t *data, uint8_t len) {
#if SELECTED_FRAMEWORK == FRAMEWORK_ARDUINO
    Wire.begin();
    Wire.beginTransmission(dev_addr);
    Wire.write(word_addr);
    Wire.endTransmission(false);
    Wire.requestFrom(dev_addr, byte(len));
    for (int i=0; i<len; i++){
        *(data+i) = Wire.read();
    }
    Wire.endTransmission();
#endif
}

void bq25730_init(bq25730_config_t *cfg) {
#if SELECTED_FRAMEWORK == FRAMEWORK_ARDUINO
    Wire.setSCL(cfg->pin_i2c_scl);
    Wire.setSDA(cfg->pin_i2c_sda);
#endif

    // Disable Low-power & Disable watchdog on ChargeOption0
    bq25730_set_watchdog(cfg);
    bq25730_lowpwr_off(cfg);

    // Enable ADC and set ADC conversion mode as continuous-mode
    bq25730_adc_enable_all(cfg);
    bq25730_adc_setmode(cfg);

    // Enable IBAT buffer to measure battery current
    bq25730_ibat_on(cfg);

    // Set a new VSYSMIN value
    bq25730_set_vsysmin(cfg);
}

void bq25730_lowpwr_on(bq25730_config_t *cfg) {
    uint8_t databuf[2];
    // Read current value of ChargeOption0
    i2c_read_registers(cfg->dev_addr, ADDR_CHRGOPT0, databuf, 2);
    delay(10);
    // Enable low-power mode to ChargeOption0
    databuf[1] |= (1 << CHRGOPT0_EN_LWPWR);  
    i2c_write_registers(cfg->dev_addr, ADDR_CHRGOPT0, databuf, 2);
    delay(10);
}

void bq25730_lowpwr_off(bq25730_config_t *cfg) {
    uint8_t databuf[2];
    // Read current value of ChargeOption0
    i2c_read_registers(cfg->dev_addr, ADDR_CHRGOPT0, databuf, 2);
    delay(10);
    // Disable low-power mode to ChargeOption0
    databuf[1] &= ~(1 << CHRGOPT0_EN_LWPWR);  
    i2c_write_registers(cfg->dev_addr, ADDR_CHRGOPT0, databuf, 2);
    delay(10);
}

void bq25730_set_watchdog(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current value of ChargeOption0 (2nd byte)
    i2c_read_registers(cfg->dev_addr, ADDR_CHRGOPT0+1, &databuf, 1);
    delay(10);
    databuf &= ~(3 << CHRGOPT0_WDTMR_ADJ);
    databuf |= (cfg->watchdog_adj << CHRGOPT0_WDTMR_ADJ);
    i2c_write_registers(cfg->dev_addr, ADDR_CHRGOPT0+1, &databuf, 1);
    delay(10);
}

void bq25730_adc_enable_all(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Enable ADC for all inputs
    databuf = databuf | 0xFF; 
    i2c_write_registers(cfg->dev_addr, ADDR_ADCOPT, &databuf, 1);
    delay(10);
}

void bq25730_adc_setmode(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCOPT (2nd byte)
    i2c_read_registers(cfg->dev_addr, ADDR_ADCOPT+1, &databuf, 1);
    delay(10);

    if (cfg->adc_mode == ADC_CONV_ONESHOT) {
        databuf &= ~(1 << ADCOPT_ADC_CONV);
    }
    else if (cfg->adc_mode == ADC_CONV_CONT) {
        databuf |= (1 << ADCOPT_ADC_CONV);
    }
    // Set ADC conversion mode
    i2c_write_registers(cfg->dev_addr, ADDR_ADCOPT+1, &databuf, 1);
    delay(10);
}

void bq25730_adc_start_conversion(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCOPT (2nd byte)
    i2c_read_registers(cfg->dev_addr, ADDR_ADCOPT+1, &databuf, 1);
    delay(10);
    // Set start ADC flag
    databuf |= (1 << ADCOPT_ADC_START);
    i2c_write_registers(cfg->dev_addr, ADDR_ADCOPT+1, &databuf, 1);
    delay(10);
}

float bq25730_read_vbus(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCVBUS
    i2c_read_registers(cfg->dev_addr, ADDR_ADCVBUS, &databuf, 1);
    delay(10);
    return (float)(VBUS_LSB * databuf);
}

float bq25730_read_vsys(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCVBUS
    i2c_read_registers(cfg->dev_addr, ADDR_ADCVSYS, &databuf, 1);
    delay(10);
    return (float)(VSYS_LSB * databuf) + VSYS_OFFSET;
}

float bq25730_read_vbat(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCVBUS
    i2c_read_registers(cfg->dev_addr, ADDR_ADCVBAT, &databuf, 1);
    delay(10);
    return (float)(VBAT_LSB * databuf) + VBAT_OFFSET;
}

float bq25730_read_vsysmin(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCVBUS
    i2c_read_registers(cfg->dev_addr, ADDR_VSYSMIN, &databuf, 1);
    delay(10);
    return (float)(VSYSMIN_LSB * databuf);
}

void bq25730_set_vsysmin(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCVBUS
    databuf = (uint8_t)(cfg->vsysmin / VSYSMIN_LSB);
    i2c_write_registers(cfg->dev_addr, ADDR_VSYSMIN, &databuf, 1);
    delay(10);
}

void bq25730_set_rsense(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current value of ChargeOption1 (2nd byte)
    i2c_read_registers(cfg->dev_addr, ADDR_CHRGOPT1+1, &databuf, 1);
    delay(10);
    // Set RNS_RSR and RSNS_RAC to RSNS_5MOHM or RSNS_10MOHM
    databuf &= ~((1 << CHRGOPT1_RSNS_RSR) | (1 << CHRGOPT1_RSNS_RAC));
    databuf |= (cfg->rsr << CHRGOPT1_RSNS_RSR);
    databuf |= (cfg->rac << CHRGOPT1_RSNS_RAC);
    i2c_write_registers(cfg->dev_addr, ADDR_CHRGOPT1+1, &databuf, 1);
    delay(10);
}

void bq25730_ibat_on(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current value of ChargeOption1 (2nd byte)
    i2c_read_registers(cfg->dev_addr, ADDR_CHRGOPT1+1, &databuf, 1);
    delay(10);
    // Enable IBAT at ChargeOption1
    databuf |= (1 << CHRGOPT1_EN_IBAT);  
    i2c_write_registers(cfg->dev_addr, ADDR_CHRGOPT1+1, &databuf, 1);
    delay(10);
}

void bq25730_ibat_off(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current value of ChargeOption1 (2nd byte)
    i2c_read_registers(cfg->dev_addr, ADDR_CHRGOPT1+1, &databuf, 1);
    delay(10);
    // Enable IBAT at ChargeOption1
    databuf &= ~(1 << CHRGOPT1_EN_IBAT);  
    i2c_write_registers(cfg->dev_addr, ADDR_CHRGOPT1+1, &databuf, 1);
    delay(10);
}

void bq25730_read_ibat(bq25730_config_t *cfg, float *ibat_charge, float *ibat_discharge) {
    uint8_t databuf[2];
    float lsb_charge;
    float lsb_discharge;
    // Read charging & discharging current of ADCIBAT 
    i2c_read_registers(cfg->dev_addr, ADDR_ADCIDCHG, databuf, 2);
    delay(10);
    if(cfg->rsr == RSNS_10MOHM){
        lsb_charge = ICHG_10MOHM_LSB; 
        lsb_discharge = IDCHG_10MOHM_LSB; 
    }
    else if(cfg->rsr == RSNS_5MOHM){
        lsb_charge = ICHG_5MOHM_LSB; 
        lsb_discharge = IDCHG_5MOHM_LSB; 
    }
    *ibat_charge = (float)(databuf[1] * lsb_charge);
    *ibat_discharge = (float)(databuf[0] * lsb_discharge);
}

float bq25730_read_iin(bq25730_config_t *cfg) {
    uint8_t databuf;
    // Read current ADCVBUS
    i2c_read_registers(cfg->dev_addr, ADDR_ADCIIN, &databuf, 1);
    delay(10);
    if(cfg->rac == RSNS_10MOHM){
        return (float)(IIN_10MOHM_LSB * databuf);
    }
    else{
        return (float)(IIN_5MOHM_LSB * databuf);
    }
}
