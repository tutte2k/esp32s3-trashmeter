#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <string.h>
#include "freertos/event_groups.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "nvs.h"

#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_SCL_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

#define ACCEL_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

#define GPIO_STEP_1 14
#define GPIO_STEP_2 13
#define GPIO_STEP_3 12
#define GPIO_STEP_4 11

const int step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}};

#define DELAY_MS 10

#define LED_PIN 18
#define LED_COUNT 8
#define RMT_CHANNEL RMT_CHANNEL_0

#define T1H 32
#define T1L 18
#define T0H 16
#define T0L 34

static rmt_item32_t items[LED_COUNT * 24];
static uint8_t led_buffer[LED_COUNT][3] = {0};

QueueHandle_t accel_data_queue;
EventGroupHandle_t accel_event_group;
#define POSITIVE_Y_EVENT BIT0
#define NEGATIVE_Y_EVENT BIT1
#define NEUTRAL_Y_EVENT BIT2

static const char *TAG = "Trashcannon";

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ};
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}
esp_err_t write_register(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ACCEL_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ACCEL_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ACCEL_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
void stepper_motor_init()
{
    ESP_LOGI(TAG, "Initializing stepper motor...");
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_STEP_1) | (1ULL << GPIO_STEP_2) | (1ULL << GPIO_STEP_3) | (1ULL << GPIO_STEP_4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}
void stepper_motor_step(int step)
{
    gpio_set_level(GPIO_STEP_1, step_sequence[step][0]);
    gpio_set_level(GPIO_STEP_2, step_sequence[step][1]);
    gpio_set_level(GPIO_STEP_3, step_sequence[step][2]);
    gpio_set_level(GPIO_STEP_4, step_sequence[step][3]);
}
void byte_to_rmt(uint8_t byte, rmt_item32_t *item)
{
    for (int i = 0; i < 8; i++)
    {
        if (byte & (0x80 >> i))
        {
            item[i].duration0 = T1H;
            item[i].level0 = 1;
            item[i].duration1 = T1L;
            item[i].level1 = 0;
        }
        else
        {
            item[i].duration0 = T0H;
            item[i].level0 = 1;
            item[i].duration1 = T0L;
            item[i].level1 = 0;
        }
    }
}
bool are_all_leds_blue()
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        if (led_buffer[i][0] != 0 || led_buffer[i][1] != 0 || led_buffer[i][2] == 0)
        {
            return false;
        }
    }
    return true;
}
void save_led_buffer_to_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error opening NVS handle!");
        return;
    }

    size_t buffer_size = LED_COUNT * 3;
    err = nvs_set_blob(nvs_handle, "led_buffer", led_buffer, buffer_size);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error saving LED buffer!");
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error committing changes to NVS!");
    }

    nvs_close(nvs_handle);
}
void load_led_buffer_from_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error opening NVS handle!");
        return;
    }

    size_t buffer_size = LED_COUNT * 3;
    err = nvs_get_blob(nvs_handle, "led_buffer", led_buffer, &buffer_size);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW("NVS", "LED buffer not found, initializing with zeros.");
        memset(led_buffer, 0, sizeof(led_buffer));
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error reading LED buffer from NVS!");
    }

    nvs_close(nvs_handle);
}
void send_color(uint8_t red, uint8_t green, uint8_t blue, bool shift_right)
{
    if (shift_right)
    {
        for (int i = LED_COUNT - 1; i > 0; i--)
        {
            memcpy(led_buffer[i], led_buffer[i - 1], 3);
        }
        led_buffer[0][0] = red;
        led_buffer[0][1] = green;
        led_buffer[0][2] = blue;
    }
    else
    {
        for (int i = 0; i < LED_COUNT - 1; i++)
        {
            memcpy(led_buffer[i], led_buffer[i + 1], 3);
        }
        led_buffer[LED_COUNT - 1][0] = red;
        led_buffer[LED_COUNT - 1][1] = green;
        led_buffer[LED_COUNT - 1][2] = blue;
    }

    int index = 0;
    for (int i = 0; i < LED_COUNT; i++)
    {
        byte_to_rmt(led_buffer[i][1], &items[index]);
        index += 8;
        byte_to_rmt(led_buffer[i][0], &items[index]);
        index += 8;
        byte_to_rmt(led_buffer[i][2], &items[index]);
        index += 8;
    }
    ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, items, LED_COUNT * 24, true));
    save_led_buffer_to_nvs();
}
void setup_led()
{
    ESP_LOGI(TAG, "Initializing led strip...");
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_PIN, RMT_CHANNEL);
    config.clk_div = 2;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(RMT_CHANNEL_0, 0, 0));
}
void task_accelerometer(void *arg)
{
    uint8_t rawData[6];
    int16_t accelX;
    int16_t accelY = 0;
    i2c_master_init();

    ESP_LOGI(TAG, "Initializing Accelerometer...");
    if (write_register(PWR_MGMT_1, 0x00) == ESP_OK)
    {
        ESP_LOGI(TAG, "Accelerometer is now active.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to wake up Accelerometer!");
        vTaskDelete(NULL);
    }

    while (1)
    {

        if (read_registers(ACCEL_XOUT_H, rawData, 6) == ESP_OK)
        {

            accelX = (rawData[0] << 8) | rawData[1];
            accelY = (rawData[2] << 8) | rawData[3];

            if (!xQueueSend(accel_data_queue, &accelX, pdMS_TO_TICKS(100)))
            {
                ESP_LOGW(TAG, "Accel data queue full!");
            }

            if (accelY > 2000)
            {
                xEventGroupSetBits(accel_event_group, POSITIVE_Y_EVENT);
            }
            else if (accelY < -2000)
            {
                xEventGroupSetBits(accel_event_group, NEGATIVE_Y_EVENT);
            }
            else
            {
                xEventGroupSetBits(accel_event_group, NEUTRAL_Y_EVENT);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read accelerometer data!");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void delayMicroseconds(uint32_t us)
{
    int64_t start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) < us)
    {
        //  ( ͡° ͜ʖ ͡°)
    }
}
int get_color_index(const char *color)
{
    if (strcmp(color, "red") == 0)
        return 0;
    else if (strcmp(color, "green") == 0)
        return 1;
    else if (strcmp(color, "blue") == 0)
        return 2;
    else
        return -1;
}
bool is_led_dominant_color(int led_index, const char *color)
{
    if (led_index < 0 || led_index >= LED_COUNT)
    {
        return false;
    }

    int color_index = get_color_index(color);
    if (color_index == -1)
    {
        return false;
    }

    return led_buffer[led_index][color_index] > led_buffer[led_index][(color_index + 1) % 3] &&
           led_buffer[led_index][color_index] > led_buffer[led_index][(color_index + 2) % 3];
}
void task_stepper_motor(void *arg)
{
    int16_t accelX;
    int step = 0;
    stepper_motor_init();

    while (1)
    {
        if (xQueueReceive(accel_data_queue, &accelX, portMAX_DELAY))
        {
            int tilt_magnitude = abs(accelX);

            if (tilt_magnitude < 100)
            {
                ESP_LOGI(TAG, "Tilt too small, motor idle.");
                continue;
            }
            if (is_led_dominant_color(LED_COUNT - 1, "blue"))
            {
                ESP_LOGI(TAG, "Blue LED detected, skipping stepper motor movement.");
                if (are_all_leds_blue())
                {
                    const int sleep_duration = 500000;
                    esp_sleep_enable_timer_wakeup(sleep_duration);
                    ESP_LOGI(TAG, "Entering light sleep... Starting RTC timer for %d microseconds", sleep_duration);
                    ESP_ERROR_CHECK(esp_light_sleep_start());
                }

                continue;
            }

            int steps_to_take = tilt_magnitude / 100;
            steps_to_take = steps_to_take > 50 ? 50 : steps_to_take;

            bool override_direction = false;
            bool direction_cw = accelX > 0;

            if (is_led_dominant_color(0, "green"))
            {
                direction_cw = false;
                override_direction = true;
            }
            else
            {
                direction_cw = true;
                override_direction = true;
            }

            for (int i = 0; i < steps_to_take; i++)
            {

                if (!override_direction)
                {
                    step = accelX > 0 ? (step + 1) % 8 : (step - 1 + 8) % 8;
                }
                else
                {
                    step = direction_cw ? (step + 1) % 8 : (step - 1 + 8) % 8;
                }

                stepper_motor_step(step);
                delayMicroseconds(3000);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}
void task_led(void *arg)
{
    setup_led();

    while (1)
    {
        EventBits_t bits = xEventGroupWaitBits(
            accel_event_group,
            POSITIVE_Y_EVENT | NEGATIVE_Y_EVENT | NEUTRAL_Y_EVENT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY);

        if (bits & POSITIVE_Y_EVENT)
        {
            send_color(0, 255, 0, true);
        }
        else if (bits & NEGATIVE_Y_EVENT)
        {
            send_color(255, 0, 0, true);
        }
        else if (bits & NEUTRAL_Y_EVENT)
        {
            send_color(0, 0, 255, false);
        }
        else
        {
            ESP_LOGW(TAG, "No event for led received.");
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}
void nvs_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    load_led_buffer_from_nvs();
}

void app_main()
{

    accel_data_queue = xQueueCreate(10, sizeof(int16_t));
    if (!accel_data_queue)
    {
        ESP_LOGE(TAG, "Failed to create accel data queue!");
        return;
    }

    accel_event_group = xEventGroupCreate();
    if (accel_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group!");
        return;
    }

    nvs_init();
    xTaskCreate(task_accelerometer, "Accelerometer Task", 2048, NULL, 5, NULL);
    xTaskCreate(task_stepper_motor, "Stepper Motor Task", 4096, NULL, 10, NULL);
    xTaskCreate(task_led, "Led Task", 4096, NULL, 3, NULL);
}
