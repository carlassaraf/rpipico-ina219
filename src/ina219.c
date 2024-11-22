#include "ina219.h"

static ina219_status_t read_register(ina219_t ina219, uint8_t reg, uint16_t *result) {
    uint8_t buf[2];

    i2c_write_timeout_us(ina219.i2c, ina219.addr, &reg, 1, true, TIMEOUT_US);
    if(i2c_read_timeout_us(ina219.i2c, ina219.addr, buf, 2, false, TIMEOUT_US) == PICO_ERROR_TIMEOUT) {
        return INA219_TIMEOUT;
    }

    *result = (buf[0] << 8) | buf[1];
    return INA219_OK;
}

static ina219_status_t write_register(ina219_t ina219, uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
    return (ina219_status_t) i2c_write_timeout_us(ina219.i2c, ina219.addr, buf, 3, true, TIMEOUT_US);
}

ina219_status_t ina219_init(ina219_t ina219) {
    // Inicializo I2C
    i2c_init(ina219.i2c, ina219.clock);
    gpio_set_function(ina219.scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(ina219.sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(ina219.scl_pin);
    gpio_pull_up(ina219.sda_pin);

    uint16_t config = 0x399F;
    ina219_status_t status = write_register(ina219, INA219_REG_CONFIG, config);
    sleep_ms(10);
}

ina219_status_t ina219_calibrate(ina219_t ina219, float shunt_resistor_value, float max_expected_amps) {
    ina219._current_lsb = max_expected_amps / 32768.0;
    uint16_t cal_reg_value = (uint16_t)(0.04096 / (ina219._current_lsb * shunt_resistor_value));
    return write_register(ina219, INA219_REG_CALIBRATION, cal_reg_value);
}

ina219_status_t ina219_read_voltage(ina219_t ina219, float *voltage) {   
    uint16_t value;
    ina219_status_t status = read_register(ina219, INA219_REG_BUSVOLTAGE, &value);
    *voltage = (value >> 3) * 0.004;
    return status;
}

ina219_status_t ina219_read_shunt_voltage(ina219_t ina219, float *shunt_voltage) {
    int16_t value;
    ina219_status_t status = read_register(ina219, INA219_REG_SHUNTVOLTAGE, &value);
    *shunt_voltage = value * 0.01;
    return status;
}

ina219_status_t ina219_read_current(ina219_t ina219, float *current) {
    int16_t value;
    ina219_status_t status = read_register(ina219, INA219_REG_CURRENT, &value);
    *current = value * ina219._current_lsb;
    return status;
}

ina219_status_t ina219_read_power(ina219_t ina219, float *power) {
    uint16_t value;
    ina219_status_t status = read_register(ina219, INA219_REG_POWER, &value);
    *power = value * 0.02;
    return status;
}
