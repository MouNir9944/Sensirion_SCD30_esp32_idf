#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


#define SCD30_ADDR 0x61
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000 

/***************************** */

#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1
#define SCD30_CMD_READ_MEASUREMENT 0x0300

#define SCD30_WRITE_DELAY_US 20           //20000=20ms

#define SENSIRION_MAX_BUFFER_WORDS 32
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL 0x4600
#define SCD30_CMD_START_PERIODIC_MEASUREMENT 0x0010
#define SCD30_CMD_STOP_PERIODIC_MEASUREMENT 0x0104

#define SCD30_CMD_GET_DATA_READY 0x0202
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
/******************************* */
i2c_master_dev_handle_t SCD30_handle;

// Function to initialize I2C
void init_i2c()
{
    i2c_master_bus_config_t bus_config = {
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .i2c_port = I2C_MASTER_NUM,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
        .clk_source = I2C_CLK_SRC_APB,
    };

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD30_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 1000,
        .flags.disable_ack_check = false,
    };
    i2c_master_bus_handle_t i2c_master_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_master_bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus, &dev_config, &SCD30_handle));
}
// CRC Calculation for Sensirion I2C communication
uint8_t sensirion_common_generate_crc(const uint8_t *data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    for (current_byte = 0; current_byte < count; ++current_byte)
    {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}
// Fill I2C command buffer with CRC
uint16_t sensirion_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd, const uint16_t *args, uint8_t num_args)
{
    uint8_t crc;
    uint8_t i;
    uint16_t idx = 0;

    buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i)
    {
        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

        crc = sensirion_common_generate_crc((uint8_t *)&buf[idx - 2], SENSIRION_WORD_SIZE);
        buf[idx++] = crc;
    }
    return idx;
}
esp_err_t sensirion_common_check_crc(const uint8_t* data, uint16_t count,
                                  uint8_t checksum) {
    if (sensirion_common_generate_crc(data, count) != checksum)
       { return ESP_FAIL;}
    return ESP_OK;
}
esp_err_t sensirion_i2c_delayed_read_cmd(i2c_master_dev_handle_t SCD30_handle, uint16_t cmd,
                                       uint32_t delay_us, uint16_t* data_words,
                                       uint16_t num_words) {
    int16_t ret;
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, cmd, NULL, 0);
    ret = i2c_master_transmit(SCD30_handle, buf, SENSIRION_COMMAND_SIZE,-1);
    if (ret != ESP_OK)
       { return ret;}

     // Delay before reading
    if (delay_us)
    {
        vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
    }
    return i2c_master_receive(SCD30_handle, data_words, num_words,-1);
}
/*
esp_err_t sensirion_i2c_delayed_read_cmd(i2c_master_dev_handle_t SCD30_handle, uint16_t cmd,
                                       uint32_t delay_us, uint16_t* data_words,
                                       uint16_t num_words) {
    int16_t ret;
    uint8_t buf[SENSIRION_COMMAND_SIZE];
    uint8_t data[3 * num_words];
    // Prepare command buffer
    sensirion_fill_cmd_send_buf(buf, cmd, NULL, 0);
    ret = i2c_master_transmit(SCD30_handle, buf, SENSIRION_COMMAND_SIZE, 1000);
    if (ret != ESP_OK)
    {
        ESP_LOGE("TAG", "I2C write failed");
        return ret;
    }

       // Delay before reading
    if (delay_us)
    {
        vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
    }
    ret = i2c_master_receive(SCD30_handle, data, sizeof(data), 1000);
    if (ret != ESP_OK)
    {
        ESP_LOGE("TAG", "I2C read failed");
        return ret;
    }
        // Process received data
    for (uint16_t i = 0; i < num_words; i++)
    {
        uint8_t crc = sensirion_common_generate_crc(&data[i * 3], 2);
        if (crc != data[i * 3 + 2])
        {
            ESP_LOGE("TAG", "CRC mismatch");
            return ESP_ERR_INVALID_CRC;
        }
        data_words[i] = (data[i * 3] << 8) | data[i * 3 + 1];
    }
}*/
esp_err_t sensirion_i2c_write_cmd_with_args(i2c_master_dev_handle_t SCD30_handle, uint16_t command,
                                          const uint16_t* data_words,
                                          uint16_t num_words) {
    uint8_t buf[SENSIRION_MAX_BUFFER_WORDS];
    uint16_t buf_size;
    buf_size = sensirion_fill_cmd_send_buf(buf, command, data_words, num_words);
    return i2c_master_transmit(SCD30_handle, buf, buf_size,-1);
}

esp_err_t sensirion_i2c_write_cmd(i2c_master_dev_handle_t SCD30_handle, uint16_t command) {
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, command, NULL, 0);
    return i2c_master_transmit(SCD30_handle, buf, SENSIRION_COMMAND_SIZE,-1);
}
esp_err_t sensirion_i2c_read_words_as_bytes(i2c_master_dev_handle_t SCD30_handle, uint8_t* data,
                                          uint16_t num_words) {
    esp_err_t ret;
    uint16_t i, j;
    uint16_t size = num_words * (SENSIRION_WORD_SIZE + CRC8_LEN);
    uint16_t word_buf[SENSIRION_MAX_BUFFER_WORDS];
    uint8_t* const buf8 = (uint8_t*)word_buf;

    ret = i2c_master_receive(SCD30_handle, buf8, size,-1);
    if (ret != ESP_OK)
        {return ret;}

    /* check the CRC for each word */
    for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {

        ret = sensirion_common_check_crc(&buf8[i], SENSIRION_WORD_SIZE,
                                         buf8[i + SENSIRION_WORD_SIZE]);
        if (ret != ESP_OK)
               {return ret;}

        data[j++] = buf8[i];
        data[j++] = buf8[i + 1];
    }

    return ESP_OK;
}
uint32_t sensirion_bytes_to_uint32_t(const uint8_t* bytes) {
    return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
           (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

float sensirion_bytes_to_float(const uint8_t* bytes) {
    union {
        uint32_t u32_value;
        float float32;
    } tmp;

    tmp.u32_value = sensirion_bytes_to_uint32_t(bytes);
    return tmp.float32;
}
/**************************************************** */
esp_err_t scd30_read_measurement(float* co2_ppm, float* temperature,
                               float* humidity) {
    esp_err_t error;
    uint8_t data[3][4];

    error =sensirion_i2c_write_cmd(SCD30_handle, SCD30_CMD_READ_MEASUREMENT);
    if (error != ESP_OK)
       {return error;}

    error = sensirion_i2c_read_words_as_bytes(SCD30_handle, &data[0][0],
                                              SENSIRION_NUM_WORDS(data));
    if (error != ESP_OK)
         {return error;}

    *co2_ppm = sensirion_bytes_to_float(data[0]);
    *temperature = sensirion_bytes_to_float(data[1]);
    *humidity = sensirion_bytes_to_float(data[2]);

    return ESP_OK;
}
esp_err_t scd30_stop_periodic_measurement() {
    return sensirion_i2c_write_cmd(SCD30_handle,
                                   SCD30_CMD_STOP_PERIODIC_MEASUREMENT);
}
/**************************************************** */
esp_err_t scd30_set_measurement_interval(uint16_t interval_sec) {
    esp_err_t error;

    if (interval_sec < 2 || interval_sec > 1800) {
        /* out of allowable range */
        return ESP_FAIL;
    }

    error = sensirion_i2c_write_cmd_with_args(
        SCD30_handle, SCD30_CMD_SET_MEASUREMENT_INTERVAL, &interval_sec,
        SENSIRION_NUM_WORDS(interval_sec));
        vTaskDelay(pdTICKS_TO_MS(SCD30_WRITE_DELAY_US));

    return error;
}
esp_err_t scd30_start_periodic_measurement(uint16_t ambient_pressure_mbar) {
    if (ambient_pressure_mbar &&
        (ambient_pressure_mbar < 700 || ambient_pressure_mbar > 1400)) {
        /* out of allowable range */
        return ESP_FAIL;
    }

    return sensirion_i2c_write_cmd_with_args(
        SCD30_handle, SCD30_CMD_START_PERIODIC_MEASUREMENT,
        &ambient_pressure_mbar, SENSIRION_NUM_WORDS(ambient_pressure_mbar));
}
/*************************************** */
esp_err_t scd30_get_data_ready(uint16_t* data_ready) {
return sensirion_i2c_delayed_read_cmd(SCD30_handle, SCD30_CMD_GET_DATA_READY, 3000, data_ready,SENSIRION_NUM_WORDS(*data_ready));
}
// Probe the SCD30 sensor
esp_err_t scd30_probe()
{
    uint16_t data_ready;
    esp_err_t ret = scd30_get_data_ready(&data_ready);
    if (ret == ESP_OK)
    {
        ESP_LOGI("TAG", "SCD30 is ready. Data Ready: %d", data_ready);
    }
    else
    {
        ESP_LOGE("TAG", "Failed to communicate with SCD30");
    }
    return ret;
}
/***************************** */
void app_main(void)
{
    float co2_ppm, temperature, relative_humidity;
    int16_t err;
    uint16_t interval_in_seconds = 2;


    init_i2c();


    while (scd30_probe() != ESP_OK) {
        printf("SCD30 sensor probing failed\n");
        vTaskDelay(pdTICKS_TO_MS(1000));
    }
    scd30_set_measurement_interval(interval_in_seconds);
    vTaskDelay(pdMS_TO_TICKS(2000));
    scd30_start_periodic_measurement(0);
    while (1) {
        uint16_t data_ready = 0;
        uint16_t timeout = 0;

        /* Poll data_ready flag until data is available. Allow 20% more than
         * the measurement interval to account for clock imprecision of the
         * sensor.
         */
        for (timeout = 0; (100000 * timeout) < (interval_in_seconds * 1200000);
             ++timeout) {
            err = scd30_get_data_ready(&data_ready);
            if (err != ESP_OK) {
                printf("Error reading data_ready flag: %i\n", err);
            }
            if (data_ready) {
                break;
            }
                vTaskDelay(pdMS_TO_TICKS(100));

        }
        if (!data_ready) {
            printf("Timeout waiting for data_ready flag\n");
            continue;
        }

        /* Measure co2, temperature and relative humidity and store into
         * variables.
         */
       err =scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
        if (err != ESP_OK) {
            printf("error reading measurement\n");

        } else {
            printf("measured co2 concentration: %0.2f ppm, "
                   "measured temperature: %0.2f degreeCelsius, "
                   "measured humidity: %0.2f %%RH\n",
                   co2_ppm, temperature, relative_humidity);
        }
    }

    scd30_stop_periodic_measurement();
}
