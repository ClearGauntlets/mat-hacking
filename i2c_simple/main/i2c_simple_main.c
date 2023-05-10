/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define MY_SLAVE_ADDR 0x0A

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

#define LIGHTHOUSE_DECK_ADDR 0x2F

#define LH_FW_SIZE          0x020000
#define LH_FLASH_PAGE_SIZE  256
#define LH_WRITE_BUF_SIZE   (5 + 4 + LH_FLASH_PAGE_SIZE)

/* Commands */
#define LHBL_BOOT_TO_FW         0x00
#define LHBL_BL_CMD             0x01
#define LHBL_GET_VERSION        0x02
#define FLASH_CMD_READ          0x03
#define FLASH_CMD_READ_STATUS   0x05
#define FLASH_CMD_WRITE_PAGE    0x02
#define FLASH_CMD_WRITE_EN      0x06
#define FLASH_CMD_ERASE_SECTOR  0xD8
#define FLASH_CMD_WAKEUP        0xAB

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t lighthouse_deck_run_command(uint8_t command, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, LIGHTHOUSE_DECK_ADDR, &command, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
static esp_err_t lighthouse_deck_get_version(uint8_t *data, size_t len)
{
    int ret;

    uint8_t version_command = 0x02;

    // Write the command to return the version
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, LIGHTHOUSE_DECK_ADDR, &version_command, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    if (ret != 0)
        ESP_LOGE(TAG, "Could not send 0x02 to lighthouse deck.");

    return ret;
}
*/

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void app_main(void)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Scan for devices
     printf("i2c scan: \n");
     for (uint8_t i = 1; i < 127; i++)
     {
        printf("iter: 0x%2x\n", i);
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    
        if (ret == ESP_OK)
        {
            printf("Found device at: 0x%2x\n", i);
        }
    }
    printf("Done.\n");

/*
    ESP_LOGI(TAG, "Attempting boot in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Get firmware version...");
    //ESP_ERROR_CHECK(lighthouse_deck_get_version(data, 1));
    ESP_ERROR_CHECK(lighthouse_deck_run_command(LHBL_GET_VERSION, data, 1));
    ESP_LOGI(TAG, "Lighthouse Deck Bootloader Version = %X", data[0]);

    ESP_ERROR_CHECK(lighthouse_deck_run_command(LHBL_BOOT_TO_FW, data, 1));
    ESP_LOGI(TAG, "Deck booting...");

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
    */
}
