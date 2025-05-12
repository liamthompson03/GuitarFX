/*
* Copyright (c) 2016 Intel Corporation
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#define SLEEP_TIME_MS   10

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define IOEXPANDER_DEV DT_NODELABEL(ioexpander)
#define ADC_DEV DT_NODELABEL(adc_dev)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const struct i2c_dt_spec ioexpander_dev = I2C_DT_SPEC_GET(IOEXPANDER_DEV);
static const struct i2c_dt_spec adc_dev = I2C_DT_SPEC_GET(ADC_DEV);

static const struct device *ioboard = DEVICE_DT_GET(DT_NODELABEL(ioboard));
static uint8_t target_din;
static uint8_t target_dout;

int target_wreq_cb(struct i2c_target_config *config) {
        return 0;
}

int target_wrec_cb(struct i2c_target_config *config, uint8_t val) {
        target_din = val;
        return 0;
}

int target_rreq_cb(struct i2c_target_config *config, uint8_t *val) {
        *val = target_dout;
        return 0;
}

int target_rpro_cb(struct i2c_target_config *config, uint8_t *val) {
        *val = target_dout; // Value returned on repeated reads... modify if data available exceeds a byte
        return 0;
}

int target_stop_cb(struct i2c_target_config *config) {
        return 0;
}

static struct i2c_target_callbacks target_callbacks = {
        .write_requested = target_wreq_cb,
        .write_received = target_wrec_cb,
	.read_requested = target_rreq_cb,
	.read_processed = target_rpro_cb,
	.stop = target_stop_cb,
};

int main(void)
{
        uint8_t ret;
        uint8_t data_io = 0x00;
        uint8_t data_zero = 0x00;
        uint8_t data_io_old = 0x00;

        if (!gpio_is_ready_dt(&led0)) {
                return 1;
        }
        if (!gpio_is_ready_dt(&led1)) {
                return 1;
        }
        if (!device_is_ready(ioexpander_dev.bus)) {
                return 1;
        }
        if (!device_is_ready(adc_dev.bus)) {
                return 1;
        }

        ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
                return 0;
        }

        ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
                return 0;
        }

        // Initialize ADC 
        uint8_t data_adc[2] = {0x00, 0x08};
        uint8_t* data_adc_addr = data_adc;
        ret = i2c_burst_write_dt(&adc_dev, 0x82, data_adc_addr, 2);
        if (ret != 0) {
                return 1;
        }

        ret = i2c_read_dt(&ioexpander_dev, &data_zero, sizeof(data_zero));
        if(ret != 0){
                return 1;
        }

        struct i2c_target_config target_cfg = {
                .address = 0x60,
                .callbacks = &target_callbacks,
        };

        // Register I2C target device
        // if(i2c_target_register(ioboard, &target_cfg) < 0) {
        //         return 1;
        // }
                                        /* MAIN LOOP */
        while (1) {
                ret = i2c_read_dt(&ioexpander_dev, &data_io, sizeof(data_io));
                if(ret != 0){
                        return 1;
                }

                ret = i2c_burst_read_dt(&adc_dev, 0x80, data_adc_addr, 2);
                if(ret != 0){
                        return 1;
                }

                if((data_adc[0] & 0x3F) > 0x01) {
                        gpio_pin_set_dt(&led1, 1);
                } else {
                        gpio_pin_set_dt(&led1, 0);
                }

                if(data_io != data_io_old && data_io != data_zero) {
                        ret = gpio_pin_toggle_dt(&led0);
                        if (ret < 0) {
                                return 1;
                        }
                }
                data_io_old = data_io;
                k_msleep(SLEEP_TIME_MS);
        }
        return 0;
}
 