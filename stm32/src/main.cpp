#include <main.h>

#define PIN_SDA     PB7
#define PIN_SCL     PB6
#define LED_B       PE8
#define LED_G       PE11
#define LED_O       PE14
#define LED_R       PE13
#define VSYSMIN_TARGET (float)11.0

/*
 * Function prototype
 */
uint8_t i2c_scan();



uint8_t ndev=0;
bq25730_config_t chip_cfg;



void setup() {
    // Use the serial monitor & Onboard LEDs
    Serial.begin(115200);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_O, OUTPUT);

    // Setup I2C and run scanner
    Wire.setSCL(PIN_SCL);
    Wire.setSDA(PIN_SDA);
    ndev = i2c_scan();

    if (ndev > 0) {
        // BQ25730 chip configuration
        chip_cfg.pin_i2c_sda = PIN_SDA;
        chip_cfg.pin_i2c_scl = PIN_SCL;
        chip_cfg.dev_addr = BQ25730_DEFAULT_ADDR;
        chip_cfg.adc_mode = ADC_CONV_CONT;
        chip_cfg.watchdog_adj = WDTMR_ADJ_DISABLE;
        chip_cfg.rsr = RSNS_10MOHM;
        chip_cfg.rac = RSNS_10MOHM;
        chip_cfg.vsysmin = VSYSMIN_TARGET;
        bq25730_init(&chip_cfg);

        digitalWrite(LED_G, HIGH);
    }
    else{
        digitalWrite(LED_R, HIGH);
    }
    delay(1000);
}

void loop() {
    float vbus, vsys, vbat, vsysmin;
    float iin, ibat_charge, ibat_discharge;
    if(ndev > 0) {
        vbus = bq25730_read_vbus(&chip_cfg);
        vsys = bq25730_read_vsys(&chip_cfg);
        vbat = bq25730_read_vbat(&chip_cfg);
        vsysmin = bq25730_read_vsysmin(&chip_cfg);
        iin = bq25730_read_iin(&chip_cfg);
        bq25730_read_ibat(&chip_cfg, &ibat_charge, &ibat_discharge);
        
        Serial.print("VSYS_MIN=");
        Serial.print(vsysmin);
        Serial.print("v.....");
        Serial.print("VBUS=");
        Serial.print(vbus);
        Serial.print("v, ");
        Serial.print("VSYS=");
        Serial.print(vsys);
        Serial.print("v, ");
        Serial.print("VBAT=");
        Serial.print(vbat);
        Serial.print("v, ");
        Serial.print("IIN=");
        Serial.print(iin);
        Serial.print("A, ");
        Serial.print("ICHG=");
        Serial.print(ibat_charge);
        Serial.print("A, ");
        Serial.print("IDCHG=");
        Serial.print(ibat_discharge);
        Serial.println("A");
    }
    delay(5000);
}



uint8_t i2c_scan() {
    byte error, address;
    int nDevices;
    nDevices = 0;
   
    Wire.begin();
    Serial.println("Scanning...");
    for(address = 1; address < 127; address++ )
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("  !");
            nDevices++;
        }
        else if (error==4)
        {
            Serial.print("Unknown error at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.println(address,HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    }
    else {
        Serial.println("done\n");
    }
    return nDevices;
}
