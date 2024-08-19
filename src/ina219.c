#include "ina219.h"

static uint16_t read_register(ina219_t ina219, uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(ina219.i2c, ina219.addr, &reg, 1, true);
    i2c_read_blocking(ina219.i2c, ina219.addr, buf, 2, false);
    return (buf[0] << 8) | buf[1];
}

static void write_register(ina219_t ina219, uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
    i2c_write_blocking(ina219.i2c, ina219.addr, buf, 3, false);
}


void ina219_init(ina219_t ina219) {
    // Inicializo I2C
    i2c_init(ina219.i2c, ina219.clock);
    gpio_set_function(ina219.scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(ina219.sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(ina219.scl_pin);
    gpio_pull_up(ina219.sda_pin);

    uint16_t config = 0x399F;
    write_register(ina219, INA219_REG_CONFIG, config);
    sleep_ms(10);
}

void ina219_calibrate(ina219_t ina219, float shunt_resistor_value, float max_expected_amps) {
    ina219._current_lsb = max_expected_amps / 32768.0;
    uint16_t cal_reg_value = (uint16_t)(0.04096 / (ina219._current_lsb * shunt_resistor_value));
    write_register(ina219, INA219_REG_CALIBRATION, cal_reg_value);
}

float ina219_read_voltage(ina219_t ina219) {   
    uint16_t value = read_register(ina219, INA219_REG_BUSVOLTAGE);
    return (value >> 3) * 0.004;
}

float ina219_read_shunt_voltage(ina219_t ina219) {
    int16_t value = (int16_t)read_register(ina219, INA219_REG_SHUNTVOLTAGE);
    return value * 0.01;
}

float ina219_read_current(ina219_t ina219) {
    int16_t value = (int16_t)read_register(ina219, INA219_REG_CURRENT);
    return value * ina219._current_lsb;
}

float ina219_read_power(ina219_t ina219) {
    uint16_t value = read_register(ina219, INA219_REG_POWER);
    return value * 0.02;
}
