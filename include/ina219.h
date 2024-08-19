#ifndef _INA219_H_
#define _INA219_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "math.h"

#define INA219_I2C_ADDR         0x40
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t clock;
    float _current_lsb;
} ina219_t;


// Prototipos

static inline ina219_t ina219_get_default_config(void) {
    return (ina219_t) {
        .i2c = PICO_DEFAULT_I2C_INSTANCE,
        .addr = INA219_I2C_ADDR,
        .sda_pin = PICO_DEFAULT_I2C_SDA_PIN,
        .scl_pin = PICO_DEFAULT_I2C_SCL_PIN,
        .clock = 400000
    };
}

void ina219_init(ina219_t ina219);
void ina219_calibrate(ina219_t ina219, float shunt_resistor_value, float max_expected_amps);
float ina219_read_voltage(ina219_t ina219);
float ina219_read_shunt_voltage(ina219_t ina219);
float ina219_read_current(ina219_t ina219);
float ina219_read_power(ina219_t ina219);

#endif // INA219_H
