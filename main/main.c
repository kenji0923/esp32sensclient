#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "esp32sensclient";

typedef enum { STATE_PING_ACCEPTING, STATE_CONFIG, STATE_READ } client_state_t;
typedef enum { MSG_SERVER_HELLO = 0x01, MSG_CLIENT_READY = 0x02, MSG_SERVER_CONFIG = 0x03, MSG_CLIENT_DATA = 0x04, MSG_SERVER_ACK = 0x05 } msg_type_t;

typedef struct {
    uint8_t server_mac[6];
    uint32_t sleep_sec;
    uint16_t batt_cyc;
    uint16_t sht_cyc;
    uint16_t dps_cyc;
    uint16_t reconfig_cyc;
    uint32_t reconfig_to_ms;
    uint32_t config_ver;
    uint8_t batt_avg;
    uint8_t sht_avg;
    uint8_t dps_osr; 
    uint8_t dps_avg; 
} config_data_t;

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    float v_batt;
    float dps_temp;
    uint32_t timestamp;
    uint32_t config_ver;
} sensor_data_t;

static RTC_DATA_ATTR client_state_t current_state = STATE_PING_ACCEPTING;
static RTC_DATA_ATTR config_data_t active_config;
static RTC_DATA_ATTR uint32_t wake_count = 0;
static RTC_DATA_ATTR uint32_t missed_acks = 0;

#define I2C_PORT_NUM                I2C_NUM_0
#define SHT40_ADDR                  0x44
#define DPS310_ADDR                 0x77
#define BATT_ADC_CHAN               ADC_CHANNEL_2

static uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static SemaphoreHandle_t msg_sem;
static bool reconfig_requested = false;

static const int32_t dps_scaling[] = { 524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960 };

static void add_peer(const uint8_t *mac) {
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
        memcpy(peer.peer_addr, mac, 6);
        esp_now_add_peer(&peer);
    }
}

static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    uint8_t type = data[0];
    if (current_state == STATE_PING_ACCEPTING && type == MSG_SERVER_HELLO) {
        memcpy(active_config.server_mac, recv_info->src_addr, 6);
        xSemaphoreGive(msg_sem);
    } else if (current_state == STATE_CONFIG && type == MSG_SERVER_CONFIG) {
        memcpy(&active_config, data + 1, sizeof(config_data_t));
        xSemaphoreGive(msg_sem);
    } else if (current_state == STATE_READ && type == MSG_SERVER_ACK) {
        missed_acks = 0;
        if (len > 1 && data[1] == 1) reconfig_requested = true;
        xSemaphoreGive(msg_sem);
    }
}

static void i2c_bus_reset() {
    gpio_config_t io_conf = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = (1ULL << 7) | (1ULL << 6) };
    gpio_config(&io_conf);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(7, 0); esp_rom_delay_us(10);
        gpio_set_level(7, 1); esp_rom_delay_us(10);
    }
}

static void read_sensors(sensor_data_t *out) {
    // 1. SHT40
    float t_sum = 0, h_sum = 0; int t_count = 0;
    int s_avg = (active_config.sht_avg > 0) ? active_config.sht_avg : 4;
    for (int i = 0; i < s_avg; i++) {
        uint8_t cmd = 0xFD; uint8_t d[6];
        if (i2c_master_write_to_device(I2C_PORT_NUM, SHT40_ADDR, &cmd, 1, 100) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(15));
            if (i2c_master_read_from_device(I2C_PORT_NUM, SHT40_ADDR, d, 6, 100) == ESP_OK) {
                t_sum += -45.0f + 175.0f * (float)((d[0]<<8)|d[1]) / 65535.0f;
                h_sum += -6.0f + 125.0f * (float)((d[3]<<8)|d[4]) / 65535.0f;
                t_count++;
            }
        }
    }
    out->temperature = (t_count > 0) ? (t_sum / t_count) : 0;
    out->humidity = (t_count > 0) ? (h_sum / t_count) : 0;

    // 2. DPS310 (Command Mode - Temp at Half OSR)
    float p_acc = 0, t_acc = 0; int p_cnt = 0;
    uint8_t p_osr = (active_config.dps_osr <= 7) ? active_config.dps_osr : 4;
    uint8_t t_osr = (p_osr > 0) ? (p_osr - 1) : 0;
    int d_avg = (active_config.dps_avg > 0) ? active_config.dps_avg : 1;
    float p_sc = (float)dps_scaling[p_osr];
    float t_sc = (float)dps_scaling[t_osr];

    uint8_t regs[18]; uint8_t c_addr = 0x10;
    if (i2c_master_write_read_device(I2C_PORT_NUM, DPS310_ADDR, &c_addr, 1, regs, 18, 100) == ESP_OK) {
        int16_t c0 = (regs[0]<<4)|(regs[1]>>4); if(c0&0x0800) c0|=0xF000;
        int16_t c1 = ((regs[1]&0x0F)<<8)|regs[2]; if(c1&0x0800) c1|=0xF000;
        int32_t c00 = (regs[3]<<12)|(regs[4]<<4)|(regs[5]>>4); if(c00&0x080000) c00|=0xFFF00000;
        int32_t c10 = ((regs[5]&0x0F)<<16)|(regs[6]<<8)|regs[7]; if(c10&0x080000) c10|=0xFFF00000;
        int16_t c01 = (regs[8]<<8)|regs[9]; int16_t c11 = (regs[10]<<8)|regs[11];
        int16_t c20 = (regs[12]<<8)|regs[13]; int16_t c21 = (regs[14]<<8)|regs[15];
        int16_t c30 = (regs[16]<<8)|regs[17];

        i2c_master_write_to_device(I2C_PORT_NUM, DPS310_ADDR, (uint8_t[]){0x06, p_osr}, 2, 100); 
        i2c_master_write_to_device(I2C_PORT_NUM, DPS310_ADDR, (uint8_t[]){0x07, t_osr | 0x80}, 2, 100); 
        i2c_master_write_to_device(I2C_PORT_NUM, DPS310_ADDR, (uint8_t[]){0x09, (p_osr > 3 ? 0x04 : 0) | (t_osr > 3 ? 0x08 : 0)}, 2, 100); 

        for (int i = 0; i < d_avg; i++) {
            i2c_master_write_to_device(I2C_PORT_NUM, DPS310_ADDR, (uint8_t[]){0x08, 0x02}, 2, 100); 
            vTaskDelay(pdMS_TO_TICKS(10 << t_osr >> 2)); // Simple dynamic wait
            i2c_master_write_to_device(I2C_PORT_NUM, DPS310_ADDR, (uint8_t[]){0x08, 0x01}, 2, 100);
            vTaskDelay(pdMS_TO_TICKS(10 << p_osr >> 2));

            uint8_t raw[6]; uint8_t p_a = 0x00;
            if (i2c_master_write_read_device(I2C_PORT_NUM, DPS310_ADDR, &p_a, 1, raw, 6, 100) == ESP_OK) {
                int32_t pr = (raw[0]<<16)|(raw[1]<<8)|raw[2]; if(pr&0x800000) pr|=0xFF000000;
                int32_t tr = (raw[3]<<16)|(raw[4]<<8)|raw[5]; if(tr&0x800000) tr|=0xFF000000;
                float tr_sc = (float)tr / t_sc; float pr_sc = (float)pr / p_sc;
                t_acc += (float)c0 * 0.5f + (float)c1 * tr_sc;
                p_acc += (float)c00 + pr_sc*(c10 + pr_sc*(c20 + pr_sc*c30)) + tr_sc*c01 + tr_sc*pr_sc*(c11 + pr_sc*c21);
                p_cnt++;
            }
        }
    }
    out->pressure = (p_cnt > 0) ? (p_acc / p_cnt) : 0;
    out->dps_temp = (p_cnt > 0) ? (t_acc / p_cnt) : 0;

    // 3. ADC
    adc_oneshot_unit_handle_t h; adc_oneshot_unit_init_cfg_t ic = {.unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&ic, &h);
    adc_oneshot_chan_cfg_t cc = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12};
    adc_oneshot_config_channel(h, BATT_ADC_CHAN, &cc);
    adc_cali_handle_t ch = NULL;
    adc_cali_curve_fitting_config_t cal_cfg = {.unit_id = ADC_UNIT_1, .chan = BATT_ADC_CHAN, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_cali_create_scheme_curve_fitting(&cal_cfg, &ch);
    int r_v, mv; uint32_t b_sum = 0;
    int b_avg = (active_config.batt_avg > 0) ? active_config.batt_avg : 16;
    for(int i=0; i<b_avg; i++) { 
        adc_oneshot_read(h, BATT_ADC_CHAN, &r_v); 
        if (ch) adc_cali_raw_to_voltage(ch, r_v, &mv); else mv = r_v * 2500 / 4095;
        b_sum += mv; vTaskDelay(1); 
    }
    out->v_batt = (2.0f * (float)b_sum / b_avg) / 1000.0f;
    if (ch) adc_cali_delete_scheme_curve_fitting(ch);
    adc_oneshot_del_unit(h);
}

void app_main(void) {
    ESP_LOGI(TAG, "BOOT... 5s wait");
    vTaskDelay(pdMS_TO_TICKS(5000));
    nvs_flash_init(); esp_netif_init(); esp_event_loop_create_default();
    wifi_init_config_t w_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&w_cfg); esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_start();
    esp_now_init(); esp_now_register_recv_cb(esp_now_recv_cb);
    msg_sem = xSemaphoreCreateBinary();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
        current_state = STATE_PING_ACCEPTING;
        wake_count = 0;
    }

    if (current_state == STATE_PING_ACCEPTING) {
        ESP_LOGI(TAG, "STATE: PING");
        add_peer(broadcast_mac);
        if (xSemaphoreTake(msg_sem, pdMS_TO_TICKS(5000)) == pdTRUE) current_state = STATE_CONFIG;
    }

    if (current_state == STATE_CONFIG) {
        ESP_LOGI(TAG, "STATE: CONFIG");
        add_peer(active_config.server_mac);
        uint8_t msg = MSG_CLIENT_READY;
        esp_now_send(active_config.server_mac, &msg, 1);
        if (xSemaphoreTake(msg_sem, pdMS_TO_TICKS(10000)) == pdTRUE) {
            ESP_LOGI(TAG, "Config OK. Ver %u", (unsigned int)active_config.config_ver);
            current_state = STATE_READ; wake_count = 0;
        } else current_state = STATE_PING_ACCEPTING;
    }

    if (current_state == STATE_READ) {
        wake_count++;
        i2c_bus_reset();
        i2c_config_t conf; memset(&conf, 0, sizeof(i2c_config_t));
        conf.mode = I2C_MODE_MASTER; conf.sda_io_num = 6; conf.scl_io_num = 7;
        conf.sda_pullup_en = 1; conf.scl_pullup_en = 1; conf.master.clk_speed = 100000;
        if (i2c_param_config(I2C_PORT_NUM, &conf) == ESP_OK && i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50));
            sensor_data_t data = {.config_ver = active_config.config_ver, .timestamp = wake_count};
            read_sensors(&data);
            uint8_t buf[sizeof(sensor_data_t) + 1];
            buf[0] = MSG_CLIENT_DATA; memcpy(buf + 1, &data, sizeof(sensor_data_t));
            add_peer(active_config.server_mac);
            esp_now_send(active_config.server_mac, buf, sizeof(buf));
            ESP_LOGI(TAG, "SENT: SHT_T:%.2f, DPS_T:%.2f, P:%.2f", data.temperature, data.dps_temp, data.pressure);
            if (xSemaphoreTake(msg_sem, pdMS_TO_TICKS(active_config.reconfig_to_ms)) != pdTRUE) {
                if (++missed_acks >= 5) current_state = STATE_PING_ACCEPTING;
            } else if (reconfig_requested) {
                current_state = STATE_CONFIG; reconfig_requested = false;
            }
            i2c_driver_delete(I2C_PORT_NUM);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    uint32_t sleep_time = (current_state == STATE_READ) ? active_config.sleep_sec : 5;
    ESP_LOGI(TAG, "Sleep: %u s", (unsigned int)sleep_time);
    esp_deep_sleep(sleep_time * 1000000ULL);
}
