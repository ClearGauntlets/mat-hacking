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
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "MAT Hacking";

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
#define LHBL_ENABLE_UART        0xBC

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define BUF_SIZE (128)

#define UART_FRAME_LENGTH 12

/**
 * @brief Data for one pulse, detected by one sensor and decoded by the FPGA on the dack.
 * Used both for lighthouse V1 and V2.
 *
 */
typedef struct {
  uint8_t sensor;
  uint32_t timestamp;

  // V1 base station data --------
  uint16_t width;

  // V2 base station data --------
  uint32_t beamData;
  uint32_t offset;
  // Channel is zero indexed (0-15) here, while it is one indexed in the base station config (1 - 16)
  uint8_t channel; // Valid if channelFound is true
  uint8_t slowBit; // Valid if channelFound is true
  bool channelFound;
} pulseProcessorFrame_t;

typedef struct {
  bool isSyncFrame;
  pulseProcessorFrame_t data;
} lighthouseUartFrame_t;

static void wait_for_uart_sync_frame() {
  char c;
  int syncCounter = 0;
  bool synchronized = false;

  uint32_t tick = portTICK_PERIOD_MS;
  ESP_LOGD(TAG, "Waiting for sync frame... UART timeout is %ld", tick);

  while (!synchronized) {
    int byte = uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t*)&c, 1, tick);
    if (byte != 1)
        ESP_LOGW(TAG, "Read %d bytes instead of 1.", byte);

    //printf("%02x", c);
    //ESP_LOGD(TAG, "%02x", c);

    if ((unsigned char)c == 0xff) {
      syncCounter += 1;
      //ESP_LOGD(TAG, "Got sync byte (%d)", syncCounter);
    } else {
      if (syncCounter > 4)
        ESP_LOGE(TAG, "Sync broken (was at %d). Restarting.", syncCounter);

      syncCounter = 0;
    }
    synchronized = (syncCounter == UART_FRAME_LENGTH);
  }
  ESP_LOGI(TAG, "Synchronized.");

}

static bool get_uart_frame_raw(lighthouseUartFrame_t *frame) {
  static char data[UART_FRAME_LENGTH];
  int syncCounter = 0;

  for(int i = 0; i < UART_FRAME_LENGTH; i++) {
    uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t*)&data[i], 1, portTICK_PERIOD_MS);
    if ((unsigned char)data[i] == 0xff) {
      syncCounter += 1;
    }
  }

  // Print out the data
  //output[len] = '\0';
  for (size_t i = 1; i < sizeof(data); ++i) printf("%02x", data[i]);
  printf(" ");

  memset(frame, 0, sizeof(*frame));

  frame->isSyncFrame = (syncCounter == UART_FRAME_LENGTH);

  frame->data.sensor = data[0] & 0x03;
  frame->data.channelFound = (data[0] & 0x80) == 0;
  frame->data.channel = (data[0] >> 3) & 0x0f;
  frame->data.slowBit = (data[0] >> 2) & 0x01;
  memcpy(&frame->data.width, &data[1], 2);
  memcpy(&frame->data.offset, &data[3], 3);
  memcpy(&frame->data.beamData, &data[6], 3);
  memcpy(&frame->data.timestamp, &data[9], 3);

  // Offset is expressed in a 6 MHz clock, convert to the 24 MHz that is used for timestamps
  frame->data.offset *= 4;

  bool isPaddingZero = (((data[5] | data[8]) & 0xfe) == 0);
  bool isFrameValid = (isPaddingZero || frame->isSyncFrame);

  //STATS_CNT_RATE_EVENT_DEBUG(&serialFrameRate);

  if (!isFrameValid) {
    ESP_LOGW(TAG, "syncCounter = %d. isPaddingZero = %d", syncCounter, isPaddingZero);
  }
  return isFrameValid;
}

static esp_err_t lighthouse_deck_run_command(uint8_t command, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, LIGHTHOUSE_DECK_ADDR, &command, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t lighthouse_deck_run_command_no_return(uint8_t command)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, LIGHTHOUSE_DECK_ADDR, &command, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

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

void debug_dump() {
  // Debugging
  static char data_debug[UART_FRAME_LENGTH];
  while (1) {
    uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t*)&data_debug, UART_FRAME_LENGTH, (20 / portTICK_PERIOD_MS) * 12);

    ESP_LOGD(TAG, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", data_debug[0],data_debug[1],data_debug[2],data_debug[3],data_debug[4],data_debug[5],data_debug[6],data_debug[7],data_debug[8],data_debug[9],data_debug[10],data_debug[11]);
    
    memset(data_debug, 0, UART_FRAME_LENGTH);

    //for (size_t i = 1; i < sizeof(data_debug); ++i) printf("%02x", data_debug[i]);
    //printf(" ");
  }
}

void fpga_data_task(void* arg){
    lighthouseUartFrame_t frame;
    bool uart_synced = false;
    bool frame_valid = false;

    debug_dump(); 

    while (1) {
        wait_for_uart_sync_frame();
        uart_synced = true;
        bool previous_frame_was_sync_frame = false;

        while ((frame_valid = get_uart_frame_raw(&frame))) {
            if (frame.isSyncFrame && previous_frame_was_sync_frame)
                ESP_LOGW(TAG, "Oops, all sync frames!");
            else if (!frame.isSyncFrame) {
                ESP_LOGI(TAG, "Timestamp: %ld, Width: %d", frame.data.timestamp, frame.data.width);
            }
            previous_frame_was_sync_frame = frame.isSyncFrame;
        }
        ESP_LOGW(TAG, "Uart not synchronized!");
        uart_synced = false;
    }
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

    printf("Configuring UART... (BAUD rate is %d)\n", ECHO_UART_BAUD_RATE);
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_REF_TICK,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    ESP_LOGI(TAG, "Get firmware version...");
    //ESP_ERROR_CHECK(lighthouse_deck_get_version(data, 1));
    ESP_ERROR_CHECK_WITHOUT_ABORT(lighthouse_deck_run_command(LHBL_GET_VERSION, data, 1));
    ESP_LOGI(TAG, "Lighthouse Deck Bootloader Version = %X", data[0]);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK_WITHOUT_ABORT(lighthouse_deck_run_command_no_return(LHBL_BOOT_TO_FW));
    ESP_LOGI(TAG, "Deck booting...");

    ESP_LOGI(TAG, "Entering main tracking loop...");

    xTaskCreate(fpga_data_task, "fpga_data_task", 4096, NULL, tskIDLE_PRIORITY, NULL);

    //ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    //ESP_LOGI(TAG, "I2C de-initialized successfully");
    ESP_LOGI(TAG, "Goodbye!");
}
