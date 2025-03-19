#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

void sendData(float val) {
    float data[2] = {val, val + 0.75};

    i2c_write_blocking(I2C_PORT, 1, (uint8_t*) data, sizeof(data), false);
    sleep_ms(500);
}

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 100 kHz
    i2c_init(I2C_PORT, 100*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    while (true) {
        for (uint16_t i = 1; i <= 16; i++) {
            sendData(i);
        }
    }
}
