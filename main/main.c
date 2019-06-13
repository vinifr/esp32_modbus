/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "main.h"
#include "modbus.h"

char buff[128];

void gpioConfig()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_SEL(GPIO_SENSOR_PWR) |
        GPIO_OUTPUT_SEL(GPIO_OUTPUT1) | GPIO_OUTPUT_SEL(GPIO_OUTPUT2) |
        GPIO_OUTPUT_SEL(GPIO_MODEM_DTR) | GPIO_OUTPUT_SEL(GPIO_MODEM_RTS) |
        GPIO_OUTPUT_SEL(GPIO_MODEM_CTS) | GPIO_OUTPUT_SEL(GPIO_SENMOD_PWR)
        | GPIO_OUTPUT_SEL(RS485_DE);
        //| GPIO_OUTPUT_SEL(GPIO_MODEM_PWR);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void modbusInit()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.rx_flow_ctrl_thresh = 2,
    };
     // RS485 configuration
    uart_param_config(RS485UART, &uart_config);
    // RS485 pins
    uart_set_pin(RS485UART, RS485_TX, RS485_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //uart_set_mode(RS485UART, UART_MODE_RS485_HALF_DUPLEX);

    ESP_ERROR_CHECK(uart_driver_install(RS485UART, 128 * 2, 128 * 2, 0,
                                        NULL, 0));
}

static void modbus_task()
{
    int i;
    int lenght = 0;
    uint8_t data[9] = {0x01, 0x03, 0x00, 0x64, 0x00, 0x1C, 0x05, 0xDC, 0x00};

    while (1) {
        uart_flush(RS485UART);
        memset(buff, 0, 128);
        write_frame(data, 8);
        if ((lenght=read_frame((uint8_t *)buff, 3)) > 0) {
            lenght=read_frame((uint8_t *)buff+3, buff[2]+2);
            printf("Modbus RX: ");
            for (i = 0; i < 10; i++)
                printf("%x ", buff[i]);
            putchar('\n');

            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}

void app_main()
{
    gpioConfig();
    // Mosbus init
    modbusInit();

    xTaskCreate(modbus_task, "modbus_task", 1024, 0, 6 + 1, 0);
}
