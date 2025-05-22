// @author = Hamid Jamal, https://github.com/Sparrowehawk
// An ESP32-IDE implementation of a HD44780 1602 16x2 serial LCD display
// Connected via a PCF8574T backpack

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"

esp_err_t ret;

#define LOW 0x00
#define HIGH 0x01

#define ACK 0x00
#define NACK 0x01

#define DATA_LENGTH 1 // 1 byte / 8 bit

// My A0 = A1 = A2 = 0 as the gaps on the board are not jumped
// However, I2C sweep showed this was the addr
#define I2C_RECEIVER_ADDR 0x27

// 1Mhz
#define I2C_CLK_SPEED 1000000

static const char *TAG = "i2c_protocol";

void i2c_master_init();
static esp_err_t i2c_read(size_t size, uint8_t *data);
static esp_err_t i2c_write(uint8_t *data, size_t size);

// Basic setup following ESP32 datasheet

void i2c_master_init()
{
    i2c_config_t i2c_tx_conf = {
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_MASTER,
        .master.clk_speed = I2C_CLK_SPEED};

    ret = i2c_param_config(I2C_NUM_0, &i2c_tx_conf);
    ESP_ERROR_CHECK(ret);

    ret = i2c_driver_install(I2C_NUM_0, i2c_tx_conf.mode, 0, 0, 0);
    ESP_ERROR_CHECK(ret);
}

static esp_err_t i2c_read(size_t size, uint8_t *data)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_RECEIVER_ADDR << 1) | I2C_MASTER_READ, ACK);
    if (size > 1)
    {
        i2c_master_read(cmd, data, size - 1, ACK);
    }
    i2c_master_read_byte(cmd, data + size - 1, NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write(uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_RECEIVER_ADDR << 1) | I2C_MASTER_WRITE, ACK);
    i2c_master_write(cmd, data, size, ACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Since I am using a backpack, I can really only send data in 2 x 4 bits per byte
// Therefore, split each byte accordingly to populate P4~P7
// P1 = RS
// RS essentially is command mode or data mode
// P2 = EN
// P3 = backlight
// Display latches onto data when a EN toggle is detected
// Aka, one high one low

static esp_err_t i2c_write_half(uint8_t data, uint8_t rs)
{
    uint8_t toSend = 0;

    toSend |= (data & 0x0F) << 4;                       // Shift to 4 MSB
    if (rs == 1)
    {
        toSend |= (1 << 0);                             // 1 gets put in LSB
    }
    toSend |= (1 << 2);                                 // EN high
    toSend |= (1 << 3);                                 // Backlight on P3
    i2c_write(&toSend, DATA_LENGTH);                    
    ESP_LOGI(TAG, "Sending nibble: 0x%02X (EN=1)", toSend);
    vTaskDelay(pdMS_TO_TICKS(5));                       // Small delay

    toSend &= ~(1 << 2);                                // Clear EN
    i2c_write(&toSend, DATA_LENGTH);
    ESP_LOGI(TAG, "Sending nibble: 0x%02X (EN=0)", toSend);
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

static esp_err_t i2c_write_bytes(uint8_t addr, uint8_t rs)
{
    // Send per half
    ESP_LOGI(TAG, "Splitting %02x in half", addr);
    i2c_write_half((addr >> 4), rs);                    // First 4 bits
    i2c_write_half((addr & 0x0F), rs);                  // 2nd 4 bits + bitmask of 11111111
    return ESP_OK;
}

static esp_err_t hd44780_init()
{
    // Setup to be in 4 bit mode
    // I can try to explain why, but this repo explains it better: 
    // https://github.com/duinoWitchery/hd44780/blob/master/hd44780.cpp
    // Otherwise, this setup is a replica of Figure 24 in the datasheet

    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Sending firt 0x03");
    i2c_write_half(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "Sending second 0x03");
    i2c_write_half(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "Sending third 0x03");
    i2c_write_half(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "Sending 0x02");
    i2c_write_half(0x02, 0); // confirm 4 bit
    vTaskDelay(pdMS_TO_TICKS(5));

    // N = 1, 2 line disp
    // F = 0, 5x7 display
    ESP_LOGI(TAG, "Sending 0x28");
    i2c_write_bytes(0x28, 0);

    ESP_LOGI(TAG, "Sending 0x08");
    i2c_write_bytes(0x08, 0);

    // Clear display
    i2c_write_bytes(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Set display on, D = 1 |  C = B = 0
    // i2c_write_bytes(0x0C, 0);

    // I/D = 1 || S = 0, entry mode set
    i2c_write_bytes(0x06, 0);

    i2c_write_bytes(0x02, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Set display on, D = 1 |  C = B = 0
    i2c_write_bytes(0x0C, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}

// Just something cute
esp_err_t love_heart()
{

    // Taken from https://www.quinapalus.com/hd44780udg.html

    uint8_t charmap[] = {
        0x00, // ░░░░░
        0x0A, // ░█░█░
        0x1F, // █████
        0x1F, // █████
        0x0E, // ░███░
        0x04, // ░░█░░
        0x00, // ░░░░░
        0x00  // ░░░░░
    };

    // Storing it in CGRAM 
    i2c_write_bytes(0x40 | (0 << 3), 0);       
    int i;

    for (i = 0; i < 8; i++)
    {
        i2c_write_bytes(charmap[i], 1);
    }

    return ESP_OK;
}

static esp_err_t set_cusor(uint8_t row, uint8_t col)
{
    uint8_t pos = col;
    if (row == 1)                               // If they want a 2nd row
    {
        pos += 0x40;
    }
    return i2c_write_bytes(0x80 | pos, 0);      // Setting cursor row
}

static esp_err_t hello_world()
{
    ret = love_heart();
    ESP_ERROR_CHECK(ret);
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = set_cusor(0, 0);
    ESP_ERROR_CHECK(ret);

    int i;
    char *str = "Hello World!";
    char *str2 = "- ESP32-IDE";

    for (i = 0; str[i] != '\0'; i++)
    {
        i2c_write_bytes(str[i], 1);
    }

    ret = set_cusor(1, 4);
    ESP_ERROR_CHECK(ret);
    for (i = 0; str2[i] != '\0'; i++)
    {
        i2c_write_bytes(str2[i], 1);
    }

    i2c_write_bytes(0x00, 1);                       // Love heart

    return ESP_OK;
}

void app_main(void)
{
    i2c_master_init();
    hd44780_init();

    love_heart(); 

    vTaskDelay(pdMS_TO_TICKS(10));                  // Allow time for CGRAM write

    hello_world();
}