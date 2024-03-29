/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"


static const char *I2C_TAG = "i2c";

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#include "mqtt_cred.h"


static const char *MQTT_TAG = "MQTT_EXAMPLE";

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a MPU6050 sensor for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO14 is assigned as the data signal of i2c master port
 *    GPIO2 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO14/GPIO2
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

//#define I2C_EXAMPLE_MASTER_SCL_IO           12                /*!< gpio number for I2C master clock */
//#define I2C_EXAMPLE_MASTER_SDA_IO           14               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_SCL_IO           5                /*!< gpio number for I2C master clock D1 */
#define I2C_EXAMPLE_MASTER_SDA_IO           4               /*!< gpio number for I2C master data D2 */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define MPU6050_SENSOR_ADDR                 0x48             /*!< slave address for MPU6050 sensor */

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

/**
 * Define the mpu6050 register address:
 */
#define ACCEL_XOUT_H    0x3B
#define WHO_AM_I        0x75  /*!< Command to read WHO_AM_I reg */

/**
 * STM32 I2C commands
 */
#define PING                    0xA0
#define CMD_SLAVE_GET_SENSORS   0xF0
#define CMD_TYPE_2_SLAVE        2
#define CMD_PUMP_SPEED          0xD0

/**
 * Timers
 */

#define PUMP_MAX_WORKING_TIME   pdMS_TO_TICKS(1000 /* 1 sec */ * 60/* 1 min */ * 10) // 10 min delay
#define NUM_TIMERS 5

/* An array to hold handles to the created timers. */
TimerHandle_t xTimers[NUM_TIMERS];

/**
 * MQTT DEFINES
 */
#define MQTT_BROKER_URL     MQTT_URI

#define BUFFER_SIZE         20  // 20bytes
#define CHANNELS            8
#define TOPIC_VALVE_STATUS  "gh/valve_status"
#define TOPIC_VALVE_CONTROL "gh/valve_control"
#define TOPIC_PUMP_STATUS   "gh/pump_status"
#define TOPIC_PUMP_CONTROL  "gh/pump_control"
#define TOPIC_PUMP_SPEED    "gh/pump_speed"
#define TOPIC_FLOW_LS       "gh/flow"

#define GPIO_MSP_RST    12
//#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1)) // example
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_MSP_RST)

typedef enum {
    DISCONNECTED = 0,
    CONNECTED
} mqtt_state_t;

enum xTIMERS {
    PUMP_TIMER = 0,
    WATERING_TIMER
} eTimer;


static TaskHandle_t I2C_MS_Task_Handler = NULL;
static TaskHandle_t I2C_1S_Task_Handler = NULL;
static esp_mqtt_client_handle_t client;
static mqtt_state_t mqtt_state = DISCONNECTED;
static uint8_t valve_state = 0;
static uint8_t pump_state = 0;
static uint8_t pump_speed = 0; // 0 .. 100 percents, 0 = OFF

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init() {
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 0;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write mpu6050
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
//static esp_err_t i2c_master_write_seq(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
//{
//    int ret;
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
//    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
//    i2c_master_stop(cmd);
//    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//
//    return ret;
//}
static esp_err_t
i2c_master_write_seq(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief test code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t
i2c_master_read_seq(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_init_stm32(i2c_port_t i2c_num) {
//    uint8_t cmd_data;
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_init();
//    cmd_data = 0x00;    // reset mpu6050
//    ESP_ERROR_CHECK(i2c_master_write_seq(i2c_num, PWR_MGMT_1, &cmd_data, 1));
//    cmd_data = 0x07;    // Set the SMPRT_DIV
//    ESP_ERROR_CHECK(i2c_master_write_seq(i2c_num, SMPLRT_DIV, &cmd_data, 1));
//    cmd_data = 0x06;    // Set the Low Pass Filter
//    ESP_ERROR_CHECK(i2c_master_write_seq(i2c_num, CONFIG, &cmd_data, 1));
//    cmd_data = 0x18;    // Set the GYRO range
//    ESP_ERROR_CHECK(i2c_master_write_seq(i2c_num, GYRO_CONFIG, &cmd_data, 1));
//    cmd_data = 0x01;    // Set the ACCEL range
//    ESP_ERROR_CHECK(i2c_master_write_seq(i2c_num, ACCEL_CONFIG, &cmd_data, 1));
    return ESP_OK;
}

static void i2c_task_example(void *arg) {
    uint8_t sensor_data[14];
    uint8_t who_am_i, i;
    double Temp;
    static uint32_t error_count = 0;
    int ret;

    i2c_master_init_stm32(I2C_EXAMPLE_MASTER_NUM);

    while (1) {
        who_am_i = 0;
        i2c_master_read_seq(I2C_EXAMPLE_MASTER_NUM, WHO_AM_I, &who_am_i, 1);

        if (0x68 != who_am_i) {
            error_count++;
        }

        memset(sensor_data, 0, 14);
        ret = i2c_master_read_seq(I2C_EXAMPLE_MASTER_NUM, ACCEL_XOUT_H, sensor_data, 14);

        if (ret == ESP_OK) {
            ESP_LOGI(I2C_TAG, "*******************\n");
            ESP_LOGI(I2C_TAG, "WHO_AM_I: 0x%02x\n", who_am_i);
            Temp = 36.53 + ((double) (int16_t)((sensor_data[6] << 8) | sensor_data[7]) / 340);
            ESP_LOGI(I2C_TAG, "TEMP: %d.%d\n", (uint16_t) Temp, (uint16_t)(Temp * 100) % 100);

            for (i = 0; i < 7; i++) {
                ESP_LOGI(I2C_TAG, "sensor_data[%d]: %d\n", i,
                         (int16_t)((sensor_data[i * 2] << 8) | sensor_data[i * 2 + 1]));
            }

            ESP_LOGI(I2C_TAG, "error_count: %d\n", error_count);
        } else {
            ESP_LOGE(I2C_TAG, "No ack, sensor not connected...skip...\n");
        }

        vTaskDelay(100 / portTICK_RATE_MS);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

static void i2c_task_ping(void *arg) {
    uint8_t pong;
    static uint32_t error_count = 0;
    int ret;
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    ret = i2c_master_init_stm32(I2C_EXAMPLE_MASTER_NUM);
    if (ret != ESP_OK)
        ESP_LOGE(I2C_TAG, "I2C Init ERROR \n");

    while (1) {
        pong = 0;
        ret = i2c_master_read_seq(I2C_EXAMPLE_MASTER_NUM, PING, &pong, 1);

        if (0xAA != pong) {
            error_count++;
        }

        if (ret == ESP_OK) {
            ESP_LOGI(I2C_TAG, "*******************\n");
            ESP_LOGI(I2C_TAG, "PONG: 0x%02x\n", pong);
            ESP_LOGI(I2C_TAG, "error_count: %d\n", error_count);
            ESP_LOGD(I2C_TAG, "uxHighWaterMark: %d\n", uxHighWaterMark);

            if (pong == 0xAA) {
                if (I2C_1S_Task_Handler != NULL)
                    xTaskNotifyGive(I2C_1S_Task_Handler);
//                if (I2C_1S_Task_Handler != NULL)
//                    xTaskNotifyGive(I2C_1S_Task_Handler);
            } else {
                ESP_LOGE(I2C_TAG, "PONG request wrong, reset MSP..\n");
//                vTaskDelay(100 / portTICK_RATE_MS);
//                gpio_set_level(GPIO_MSP_RST, 0);  // RST MSP
//                vTaskDelay(500 / portTICK_RATE_MS); // wait till capacitor discharge
//                gpio_set_level(GPIO_MSP_RST, 1);  // release RST PIN
//                vTaskDelay(100 / portTICK_RATE_MS); // extra delay for MSP boot process
            }
        } else {
            ESP_LOGE(I2C_TAG, "No ack, sensor not connected...restarting i2c driver...\n");
            ESP_LOGD(I2C_TAG, "uxHighWaterMark: %d\n", uxHighWaterMark);
            ret = i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
            if (ret == ESP_OK) {
                i2c_master_init_stm32(I2C_EXAMPLE_MASTER_NUM);
            } else
                ESP_LOGE(I2C_TAG, "Can't delete driver.\n");
            vTaskDelay(100 / portTICK_RATE_MS);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

static void i2c_task_ticks_ms(void *arg) {
    uint8_t data_buffer[BUFFER_SIZE];
    uint16_t data[CHANNELS];
//    uint8_t i;
//    static uint32_t error_count = 0;
    int ret;
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

//    i2c_master_init_stm32(I2C_EXAMPLE_MASTER_NUM);

    while (1) {
        xTaskNotifyWait(pdFALSE, UINT_LEAST32_MAX, NULL, portMAX_DELAY);
        ret = i2c_master_read_seq(I2C_EXAMPLE_MASTER_NUM, CMD_TYPE_2_SLAVE, data_buffer, CHANNELS * 2);

        if (ret == ESP_OK) {
            for (int j = 0; j < CHANNELS; ++j) {
                data[j] = data_buffer[j + j] << 8;
                data[j] += data_buffer[j + j + 1];
            }
//            ESP_LOGI(I2C_TAG, "*******************\n");
//            ESP_LOGI(I2C_TAG, "PONG: 0x%02x\n", pong);
            ESP_LOGI(I2C_TAG, "TICKS20MS #1: %d | #2: %d | #3: %d | #4: %d | #5: %d | #6: %d | #7: %d | #8: %d",
                     data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        } else {
            ESP_LOGE(I2C_TAG, "No ack, %d\n", ret);
        }
        ESP_LOGD(I2C_TAG, "uxHighWaterMark: %d\n", uxHighWaterMark);
        ESP_LOGD(I2C_TAG, "MinFreeHeap: %d\n", esp_get_minimum_free_heap_size());
        ESP_LOGD(I2C_TAG, "FreeHeap: %d\n", esp_get_free_heap_size());

        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

//    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

static void i2c_task_ticks_sec(void *arg) {
    uint8_t data_buffer[BUFFER_SIZE];
    uint16_t data[CHANNELS];
//    static uint32_t error_count = 0;
    int ret;
    int msg_id = (int) NULL;
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

//    i2c_master_init_stm32(I2C_EXAMPLE_MASTER_NUM);

    while (1) {
        xTaskNotifyWait(pdFALSE, UINT_LEAST32_MAX, NULL, portMAX_DELAY);
        ret = i2c_master_read_seq(I2C_EXAMPLE_MASTER_NUM, CMD_SLAVE_GET_SENSORS, data_buffer, CHANNELS * 2);

        if (ret == ESP_OK) {
            for (int j = 0; j < CHANNELS; ++j) {
                data[j] = data_buffer[j + j] << 8;
                data[j] += data_buffer[j + j + 1];
            }
//            ESP_LOGI(I2C_TAG, "*******************\n");
//            ESP_LOGI(I2C_TAG, "PONG: 0x%02x\n", pong);
            ESP_LOGI(I2C_TAG, "TICKS1SEC #1: %d | #2: %d | #3: %d | #4: %d | #5: %d | #6: %d | #7: %d | #8: %d",
                     data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        } else {
            ESP_LOGE(I2C_TAG, "No ack, sensor not connected...skip...\n");
        }

        /**
         * PUMP SPEED WRITE
         */
        ret = i2c_master_write_seq(I2C_NUM_0, CMD_PUMP_SPEED, &pump_speed, 1);
        if (ret == ESP_OK) {
            ESP_LOGI(I2C_TAG, "Write PUMP speed successfully.\n");
        } else
            ESP_LOGE(I2C_TAG, "Write PUMP speed failed.\n");
        ESP_LOGD(I2C_TAG, "uxHighWaterMark: %d\n", uxHighWaterMark);
        ESP_LOGD(I2C_TAG, "MinFreeHeap: %d\n", esp_get_minimum_free_heap_size());
        ESP_LOGD(I2C_TAG, "FreeHeap: %d\n", esp_get_free_heap_size());

        static char buff[20];
        /**
         * valve state publish
         */
        itoa(valve_state, buff, 10);
        if (mqtt_state == CONNECTED) {
            msg_id = esp_mqtt_client_publish(client, TOPIC_VALVE_STATUS, buff, 1, 0, 0);
            ESP_LOGI(MQTT_TAG, "[TOPIC_VALVE_STATUS] sent publish successful, msg_id=%d", msg_id);
        }
        ESP_LOGI(MQTT_TAG, "[CLIENT STATE] MQTT state=%d", mqtt_state);

        /**
         * sensors data publish
         */

        /*
         * flow data publish
         */

        /**
         * pump state publish
         */

        vTaskDelay(1000 / portTICK_RATE_MS);
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    client = event->client;

    int msg_id;
    char data_buff[16];
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_state = CONNECTED;
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");

//            msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
//            ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);
//
//            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
//            ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id);
//
//            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
//            ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id);
//
//            msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
//            ESP_LOGI(MQTT_TAG, "sent unsubscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, TOPIC_VALVE_CONTROL, 0);
            ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, TOPIC_PUMP_SPEED, 0);
            ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_state = DISCONNECTED;
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            strncpy(data_buff, event->data, event->data_len);
            ESP_LOGW(MQTT_TAG, "DATA= %s", data_buff);

            // TODO replace to switch or if-else to reduce calculation
            // Set valve state
            if (strncmp(event->topic, TOPIC_VALVE_CONTROL, event->topic_len) == 0) {
                if (event->data_len == 1) {
                    uint8_t data_tmp = (uint8_t) atoi(event->data);
                    if (data_tmp == 0 || data_tmp == 1)
                        valve_state = data_tmp;
                }
            }
            if (strncmp(event->topic, TOPIC_PUMP_CONTROL, event->topic_len) == 0) {
                if (event->data_len == 1) {
                    uint8_t data_tmp = (uint8_t) atoi(event->data);
                    if (data_tmp == 0 || data_tmp == 1) // can only be 0 or 1
                        pump_state = data_tmp;
                }
            }
            if (strncmp(event->topic, TOPIC_PUMP_SPEED, event->topic_len) == 0) {
                ESP_LOGW(MQTT_TAG, "PUMP SPEED detected\n");
                ESP_LOGW(MQTT_TAG, "PUMP SPEED = %d", atoi(data_buff));
                ESP_LOGW(MQTT_TAG, "PUMP SPEED data len = %d", event->data_len);
                uint8_t data_tmp = (uint8_t) atoi(data_buff);
                if (data_tmp <= 100 && data_tmp >= 0) { // 0 - 100 (%)
                    if (pump_speed == 0 && data_tmp > 0 && xTimerIsTimerActive(xTimers[PUMP_TIMER]) != pdTRUE) {
                        if (xTimerStart(xTimers[PUMP_TIMER], 0) != pdPASS) {
                            /* The timer could not be set into the Active
                            state. */
                            data_tmp = 0; // turn off pump
                        } else {
                            ESP_LOGW(MQTT_TAG, "xTimer PUMP started");
                        }
                    }
                    pump_speed = data_tmp;
                    ESP_LOGW(MQTT_TAG, "PUMP SPEED between accepted values = %d", data_tmp);
                    i2c_master_write_seq(I2C_NUM_0, CMD_PUMP_SPEED, &pump_speed, 1);
                }

            }

            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(MQTT_TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
            .uri = MQTT_BROKER_URL,
            .disable_auto_reconnect = false,
            .reconnect_timeout_ms = 10000,

    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

/* Define a callback function that will be used by multiple timer
 instances.  The callback function does nothing but count the number
 of times the associated timer expires, and stop the timer once the
 timer has expired 10 times.  The count is saved as the ID of the
 timer. */
void vTimerCallback(TimerHandle_t xTimer) {
    const uint32_t ulMaxExpiryCountBeforeStopping = 10;
    uint32_t TimID;

    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT(xTimer)

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    TimID = (uint32_t) pvTimerGetTimerID(xTimer);

    switch (TimID) {
        case PUMP_TIMER:
            ESP_LOGW(I2C_TAG, "TIMER EXPIRED, PUMP SPEED set to 0, PUMP OFF");
            pump_speed = 0;
            i2c_master_write_seq(I2C_NUM_0, CMD_PUMP_SPEED, &pump_speed, 1);
            esp_mqtt_client_publish(client, TOPIC_PUMP_SPEED, "0", 1, 0, 0);
            break;
        case WATERING_TIMER:
            //TODO close valve
            break;
        default:
            break;
    }
}


void app_main(void) {
    int ret;
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
        ESP_LOGE("GPIO Configure ERR: %s", "%d", ret);

    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(GPIO_MSP_RST, 0);  // RST MSP
    vTaskDelay(500 / portTICK_RATE_MS); // wait till capacitor discharge
    gpio_set_level(GPIO_MSP_RST, 1);  // release RST PIN
    vTaskDelay(100 / portTICK_RATE_MS); // extra delay for MSP boot process

    /* Create then start some timers.  Starting the timers before
    the RTOS scheduler has been started means the timers will start
    running immediately that the RTOS scheduler starts. */

    xTimers[PUMP_TIMER] = xTimerCreate
            ( /* Just a text name, not used by the RTOS
                     kernel. */
                    "PumpTimer",
                    /* The timer period in ticks, must be
                    greater than 0. */
                    PUMP_MAX_WORKING_TIME,
                    /* The timers will auto-reload themselves
                    when they expire. */
                    pdFALSE,
                    /* The ID is used to store a count of the
                    number of times the timer has expired, which
                    is initialised to 0. */
                    (void *) PUMP_TIMER,
                    /* Each timer calls the same callback when
                    it expires. */
                    vTimerCallback
            );
    xTimers[WATERING_TIMER] = xTimerCreate("WateringTimer", PUMP_MAX_WORKING_TIME,
                                           pdFALSE, (void *) WATERING_TIMER, vTimerCallback);

    if (xTimers[PUMP_TIMER] == NULL) {
        /* The timer was not created. */
    }
    /* Start the timer.  No block time is specified, and
    even if one was it would be ignored because the RTOS
    scheduler has not yet been started. */
//    if (xTimerStart(xTimers[PUMP_TIMER], 0) != pdPASS) {
//        /* The timer could not be set into the Active
//        state. */
//    }

    /* ...
    Create tasks here.
    ... */


    //start i2c task
//    xTaskCreate(i2c_task_example, "i2c_task_example", 2048, NULL, 10, NULL);
    xTaskCreate(i2c_task_ping, "i2c_task_example", 1024, NULL, 11, NULL);
//    xTaskCreate(i2c_task_ticks_ms, "i2c_task_ticks_ms", 2048, NULL, 10, &I2C_MS_Task_Handler);
    xTaskCreate(i2c_task_ticks_sec, "i2c_task_ticks_sec", 2048, NULL, 10, &I2C_1S_Task_Handler);

    ESP_LOGI(MQTT_TAG, "[APP] Startup..");
    ESP_LOGI(MQTT_TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(MQTT_TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO)
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE)
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE)
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE)
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE)
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE)
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE)

    ESP_ERROR_CHECK(nvs_flash_init())
    ESP_ERROR_CHECK(esp_netif_init())
    ESP_ERROR_CHECK(esp_event_loop_create_default())

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect())

    mqtt_app_start();
}